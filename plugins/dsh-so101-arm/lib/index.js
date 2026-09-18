/**
 * DS101 SO-101 arm — a host-composition plugin that gives an agent hands.
 *
 * ## Where this row belongs, and why
 *
 * This is a TOOL plugin: it publishes nothing, so it binds no service and needs no isolate
 * realm. It can sit either in the host composition (every session gets the arm) or in an
 * agent preset (one session does). The DS101 bench is a single physical resource that one
 * operator drives, so the host plane is the honest default — and it is what the deployment
 * this was written for uses.
 *
 * ## The split of responsibility
 *
 * The bridge (a Python process on loopback) owns the hardware: it is the only thing that
 * opens COM10 and the RealSense, it holds the calibration, and it enforces the joint
 * envelope, the rate limiter, the pre-flight checks and the heartbeat watchdog. This plugin
 * owns the MODEL-FACING surface: tool schemas, defaults, and the messages that turn a bridge
 * refusal into something an agent can act on.
 *
 * That split is deliberate. A tool call crosses a process boundary, so nothing here can be
 * in the control loop — which is correct, because an agent round-trip is seconds and a
 * stalled servo is milliseconds.
 *
 * ## Schema notes
 *
 * `parameters` is a `ParameterSchemaSpec`: a map of parameter NAME to property spec, which
 * the registry compiles into an implicit OPEN object root. It is not itself a schema, so it
 * must NOT carry a `type` key — adding one is a confusing error, because the validation
 * failure reads "parameters.type must be a value schema object", which sounds like the
 * opposite of the real problem. `params()` below exists to keep that explicit.
 *
 * Output roots are `ValueSchemaSpec`, where an object root DOES need `type: 'object'` and an
 * explicit `additionalProperties`.
 *
 * `test/load-check.mjs` is what catches a mistake here, so run it after any edit.
 */
import { defineTool } from '@deepseek-ai/dsh-tools'
import z from '@deepseek-ai/schemastery'
import { readFileSync, writeFileSync, statSync } from 'node:fs'
import { join } from 'node:path'
import { homedir } from 'node:os'

/** Where the bench lives. Every path is configuration so the package is not machine-bound. */
const Config = z.object({
  /** Repository root holding bridge/ and calib/. */
  workspace: z.string().default('D:\\vscode\\DS101'),
  /** Python interpreter that has pyrealsense2, opencv, pyserial and scservo_sdk. */
  python: z.string().default('D:\\vscode\\DS101\\.venv\\Scripts\\python.exe'),
  /** Loopback port the bridge listens on. */
  port: z.number().default(8731),
  /** Serial port of the SO-101 follower. */
  serialPort: z.string().default('COM10'),
  /**
   * Motion is OFF unless this is set. A plugin must never be able to energise a robot merely
   * by being installed, so launching without it produces a read-only bridge.
   */
  allowMotion: z.boolean().default(false),
  /** Table surface in the arm frame: the shoulder axis sits this far above the table. */
  tableZMm: z.number().default(-115),
  /** Measured mount geometry, used for the pre-flight framing report and the synthetic camera. */
  cameraForwardMm: z.number().default(440),
  cameraHeightMm: z.number().default(625),
  cameraPitchDeg: z.number().default(16),
  /** Seconds one bridge request may take before the tool gives up. */
  requestTimeoutS: z.number().default(120),
})

export const name = 'dsh-so101-arm'
export const inject = ['tools']

/**
 * Raw `defineTool` options, in registration order, kept only so a load test can re-validate
 * exactly what this module builds. `defineTool` returns a NORMALIZED definition whose
 * `parameters` has already been compiled into a JSON schema, so a test that re-feeds the
 * return value is checking the harness, not this plugin — this list is what makes the check
 * faithful, and it costs nothing at runtime.
 */
export const __rawToolDefinitions = []

export function apply(ctx, config) {
  const cfg = config ?? {}
  const port = cfg.port ?? 8731
  const base = `http://127.0.0.1:${port}`
  const workspace = cfg.workspace ?? 'D:\\vscode\\DS101'
  const python = cfg.python ?? 'python'
  const timeoutMs = (cfg.requestTimeoutS ?? 120) * 1000

  /**
   * The tool registry — declared in `inject` AND read through `ctx.get` with a check.
   *
   * Those two are not redundant, because they promise different things. `inject` is a promise
   * about ORDERING: Cordis activates this plugin only once `tools` exists, so on any healthy
   * profile the value below is present. `ctx.get` is a promise about the FAILURE MODE: if it
   * ever is not present, this plugin goes quiet instead of throwing.
   *
   * The distinction is not academic. A bare `ctx.tools` naming a service that is not in
   * `inject` is refused by the Cordis property guard, and that throw propagates out through
   * the profile's include row and takes the ENTIRE profile down into recovery mode. One
   * plugin must never be able to do that, so the registry is asked for rather than assumed.
   */
  const tools = ctx.get('tools')
  if (tools === undefined) {
    console.warn('[dsh-so101-arm] no tool registry in this context; the arm_* tools are not registered')
    return
  }

  const text = (value) => [{ type: 'text', text: JSON.stringify(value, null, 2) }]
  /** Output root: a ValueSchemaSpec, so an object root needs `type` and `additionalProperties`. */
  const output = () => ({
    schema: { type: 'object', additionalProperties: true },
    render: (_args, value) => text(value),
  })
  /**
   * Parameter root: a map of name -> property spec. `parameterSchemaSpecToJsonSchema` turns it
   * into an open object root, so a `type` key here is a BUG, not a redundancy.
   */
  const params = (properties) => ({ ...(properties ?? {}) })

  /** Register a tool, recording the raw options so the load test can re-validate them. */
  const tool = (options) => {
    __rawToolDefinitions.push(options)
    tools.register(defineTool(options))
  }

  /**
   * Whether the last real bridge call got an answer. Gates the keepalive below so a
   * bridge that is simply not running does not collect a request every 1.5 seconds.
   */
  let bridgeReachable = false

  /** One bridge call. Never throws for a domain failure: a refusal is information. */
  async function call(method, path, payload) {
    const controller = new AbortController()
    const timer = setTimeout(() => controller.abort(), timeoutMs)
    try {
      const res = await fetch(base + path, {
        method,
        signal: controller.signal,
        headers: payload === undefined ? undefined : { 'Content-Type': 'application/json' },
        body: payload === undefined ? undefined : JSON.stringify(payload),
      })
      const body = await res.json().catch(() => ({}))
      // A heartbeat is not evidence about the bridge: it is served outside the bench lock and
      // answers even while a call is wedged, so letting it set this would mask a dead bridge.
      if (path !== '/heartbeat') bridgeReachable = true
      return { reachable: true, httpStatus: res.status, body }
    } catch (error) {
      if (path !== '/heartbeat') bridgeReachable = false
      return {
        reachable: false,
        error: String(error?.message ?? error),
        hint:
          `The DS101 bridge is not answering on ${base}. Start it first — it owns the serial ` +
          `port and the RealSense, so exactly one instance may run:\n` +
          `  cd ${workspace}\n` +
          `  ${python} bridge\\server.py${cfg.allowMotion ? ' --allow-motion' : ''}\n` +
          `Or call arm_launch to start it from here. Stop it with a POST to ${base}/shutdown; ` +
          `never hard-kill it, or the COM port handle can stay stuck until the USB adapter is ` +
          `physically replugged.`,
      }
    } finally {
      clearTimeout(timer)
    }
  }

  /**
   * Hold the bridge's torque lease open for as long as this plugin is loaded.
   *
   * The bridge releases servo torque when no client has checked in for `--watchdog-s`, which
   * defaults to 5 seconds. That is the right default for a script, which loops tightly. It is
   * the wrong default for this harness: an agent is silent for tens of seconds between tool
   * calls, so the lease expires between every single step. These servos have no brake, so a
   * released arm sags under gravity -- the next tool call then finds it somewhere else, plans
   * from there, and a multi-step approach/grasp/lift can never converge. From outside it looks
   * exactly like "the arm does not move", and no amount of re-sending the target fixes it.
   *
   * The harness is a client, so it should behave like one and keep the lease alive. This only
   * ever POSTs /heartbeat, so it cannot start a motion and cannot undo an estop: nothing
   * re-enables torque except an explicit move or arm_enable. It is an effect, so stopping or
   * unloading this plugin drops the lease and the bridge's own watchdog takes over.
   */
  const heartbeatTimer = setInterval(() => {
    if (!bridgeReachable) return
    void call('POST', '/heartbeat', {})
  }, Math.max(200, (cfg.heartbeatS ?? 1.5) * 1000))
  if (typeof heartbeatTimer.unref === 'function') heartbeatTimer.unref()
  ctx.effect(() => () => clearInterval(heartbeatTimer))

  /**
   * Read a collected subprocess stream from the beginning.
   *
   * Collect-mode readers are offset-based and NON-consuming, so reading from offset 0 is safe
   * to repeat. Trimmed to the tail: what went wrong is nearly always the last thing printed.
   */
  function readCollected(reader, limit = 2000) {
    if (reader === undefined || reader === null) return ''
    try {
      return reader.readFrom(0).text.slice(-limit)
    } catch {
      return ''
    }
  }

  /** Map a bridge reply onto the model's view, keeping the two harness-level failures distinct. */
  function unwrap(reply) {
    if (!reply.reachable) {
      return { ok: false, bridge_reachable: false, error: 'bridge unreachable', hint: reply.hint }
    }
    const body = reply.body ?? {}
    if (body.ok === false && typeof body.error === 'string') {
      return { ...body, httpStatus: reply.httpStatus }
    }
    return body
  }

  tool({
    name: 'arm_status',
    description:
      'Read the DS101 arm state: joint angles, tool position in the arm frame, servo voltage ' +
      'and temperature, whether the bridge permits motion, and whether the bridge is reachable ' +
      'at all. Call this first in any robot turn — a read-only bridge is a configuration fact ' +
      'rather than a fault, and it will refuse every move.',
    parameters: params(),
    output: output(),
    async execute() {
      const health = await call('GET', '/health')
      if (!health.reachable) return unwrap(health)
      return { ok: true, health: health.body, state: unwrap(await call('GET', '/state')) }
    },
  })

  tool({
    name: 'arm_scene',
    description:
      'Structured perception of the bench from the fixed overhead RealSense, returned as ' +
      'NUMBERS rather than an image: the fitted table plane, and per object the centroid, ' +
      'extent, height above the table, median BGR colour and a footprint centre, all in ' +
      'arm-frame millimetres. Aim grasps at footprint_centre_base_mm rather than ' +
      'centroid_base_mm: the top face projects toward the camera, so the centroid is biased ' +
      'away from the true centre by roughly the object height.',
    parameters: params(),
    output: output(),
    async execute() {
      return unwrap(await call('POST', '/scene', {}))
    },
  })

  tool({
    name: 'arm_pixel_to_base',
    description:
      'Convert a pixel in the overhead colour image plus its depth into arm-frame millimetres ' +
      'using the hand-eye calibration. Use it when something is visible in the image but the ' +
      'object detector did not separate it. Fails with a clear reason when no extrinsics ' +
      'calibration exists yet.',
    parameters: params({
      u: { type: 'number', required: true, description: 'pixel column in the overhead image' },
      v: { type: 'number', required: true, description: 'pixel row in the overhead image' },
    }),
    output: output(),
    async execute(args) {
      return unwrap(await call('POST', '/pixel_to_base', { u: args.u, v: args.v }))
    },
  })

  tool({
    name: 'arm_plan',
    description:
      'DRY RUN: solve inverse kinematics for a Cartesian target and report the joint angles, ' +
      'residual error and reachability WITHOUT moving the arm. Always call this before ' +
      'arm_move_to — the wrist, not the tool tip, is what IK solves for, so a tip target that ' +
      'looks reachable can still be outside the envelope.',
    parameters: params({
      target_mm: {
        type: 'array',
        required: true,
        items: { type: 'number' },
        description:
          'Target tool position [x, y, z] in arm-frame millimetres. Origin is the shoulder-pan ' +
          'axis, +x away from the arm toward the workspace, +z up. The table is at negative z.',
      },
    }),
    output: output(),
    async execute(args) {
      return unwrap(await call('POST', '/plan', { target_mm: args.target_mm }))
    },
  })

  tool({
    name: 'arm_move_to',
    description:
      'Move the gripper tool point to a Cartesian target in arm-frame millimetres. Runs IK, ' +
      'then executes with the servo trajectory generator, the rate limiter and the joint ' +
      'envelope. Read `status`: "reached" means it arrived, "clamped" means the target was ' +
      'outside the envelope and it did NOT arrive, "budget_exhausted" means the 15-degree ' +
      'per-command limiter needs another call. Requires a bridge started with motion enabled.',
    parameters: params({
      target_mm: {
        type: 'array',
        required: true,
        items: { type: 'number' },
        description: 'Target tool position [x, y, z] in arm-frame millimetres.',
      },
      wait: { type: 'boolean', description: 'Wait for the move to settle. Default true.' },
    }),
    output: output(),
    async execute(args) {
      return unwrap(await call('POST', '/move_to', {
        target_mm: args.target_mm,
        wait: args.wait !== false,
      }))
    },
  })

  tool({
    name: 'arm_gripper',
    description:
      'Open or close the SO-101 gripper (0 percent = closed, 100 = open). Read possible_grasp ' +
      'from the reply: these servos have no force sensing, so a jaw that stalls short of its ' +
      'commanded position is holding something, and a jaw that reaches full travel closed on ' +
      'nothing. That stall is the only grasp feedback that exists — never infer a grasp from ' +
      'the fact that the move succeeded.',
    parameters: params({
      openness_pct: {
        type: 'number',
        required: true,
        description: '0 = fully closed, 100 = fully open.',
      },
    }),
    output: output(),
    async execute(args) {
      return unwrap(await call('POST', '/gripper', { openness_pct: args.openness_pct }))
    },
  })

  tool({
    name: 'arm_estop',
    description:
      'Emergency SOFTWARE stop: release torque on every servo immediately so the arm goes limp. ' +
      'Use it whenever anything looks wrong. This is NOT the emergency stop — that is the ' +
      'physical switch on the servo power, and it must exist before any autonomous motion. ' +
      'Re-enable with arm_enable.',
    parameters: params(),
    output: output(),
    async execute() {
      return unwrap(await call('POST', '/estop', {}))
    },
  })

  tool({
    name: 'arm_enable',
    description:
      'Re-enable servo torque after a stop, subject to the bridge pre-flight checks (voltage, ' +
      'temperature, servo error flags). A refusal here names the real problem and must not be ' +
      'worked around by raising a threshold.',
    parameters: params(),
    output: output(),
    async execute() {
      return unwrap(await call('POST', '/enable', {}))
    },
  })

  tool({
    name: 'arm_release',
    description:
      'Release servo torque so the arm can be moved by hand. Use it before any manual ' +
      'positioning, including the joint-zero calibration.',
    parameters: params(),
    output: output(),
    async execute() {
      return unwrap(await call('POST', '/release', {}))
    },
  })

  tool({
    name: 'arm_home',
    description:
      'Move the arm back to its recorded HOME pose — the tick pose the operator saved into the ' +
      'calibration file as `home_ticks`. Use it when a task is finished, or to hand the bench ' +
      'back in a configuration somebody chose on purpose. Read the reply: a status other than ' +
      '"reached" means the arm did not arrive, and the achieved angles say where it stopped.\n' +
      'This is a MOVE, not a recovery action. After a collision, an over-temperature reading, a ' +
      'servo error flag or an estop, the arm must be stopped and looked at — driving a jammed or ' +
      'overheated joint makes it worse, and motion nobody asked for is precisely what an ' +
      'emergency stop exists to prevent. Stopping belongs to arm_estop; this one moves.',
    parameters: params({
      wait: { type: 'boolean', description: 'Wait for the move to settle (default true).' },
    }),
    output: output(),
    async execute(args) {
      return unwrap(await call('POST', '/home', { wait: args.wait !== false }))
    },
  })

  tool({
    name: 'arm_launch',
    description:
      'Start the DS101 bridge as a background process, after first confirming that nothing is ' +
      'already serving. The bridge owns the serial port and the camera, so exactly one ' +
      'instance may run; this refuses rather than starting a second. Motion stays off unless ' +
      'the plugin was configured with allowMotion.',
    parameters: params({
      simulate: {
        type: 'boolean',
        description:
          'Start against simulated servos AND a synthetic camera, touching no hardware. Use ' +
          'this to rehearse a sequence before doing it for real.',
      },
    }),
    output: output(),
    async execute(args) {
      const existing = await call('GET', '/health')
      if (existing.reachable) {
        return { ok: true, already_running: true, health: existing.body }
      }
      const subprocess = ctx.get('subprocess')
      if (subprocess === undefined) {
        return {
          ok: false,
          error: 'no subprocess service in this composition',
          hint: `start it by hand:  cd ${workspace} ; ${python} bridge\\server.py`,
        }
      }
      const argv = [
        python,
        'bridge/server.py',
        '--http-port', String(port),
        '--port', cfg.serialPort ?? 'COM10',
        '--table-z-mm', String(cfg.tableZMm ?? -115),
        '--camera-forward-mm', String(cfg.cameraForwardMm ?? 440),
        '--camera-height-mm', String(cfg.cameraHeightMm ?? 625),
        '--camera-pitch-deg', String(cfg.cameraPitchDeg ?? 16),
      ]
      if (cfg.allowMotion) argv.push('--allow-motion')
      if (args.simulate) argv.push('--simulate')
      let handle
      try {
        handle = subprocess.spawn({
          argv,
          cwd: workspace,
          // Collected rather than inherited. A bridge that dies during import — a missing
          // pyrealsense2, a COM port still held by a previous process — says why on stderr,
          // and that line is the only thing that turns "it did not start" into something
          // fixable. With `inherit` the reason scrolls past in a console nobody is reading.
          stdio: {
            stdin: 'ignore',
            stdout: { maxBytes: 262144 },
            stderr: { maxBytes: 262144 },
          },
          graceMs: 3000,
        })
      } catch (error) {
        return { ok: false, error: String(error?.message ?? error), argv }
      }

      // `SubprocessHandle` has NO `pid` — only the terminal handle does. Reporting
      // `handle.pid` therefore put `undefined` into the tool result, and the harness rejected
      // the whole call as "not lossless JSON": a failure that looked like a plugin bug while
      // hiding the fact that the launch had been attempted at all.
      //
      // What the handle does offer is `done`. Waiting out the window in which a wrong
      // interpreter path or an unimportable module exits turns an optimistic "starting" into
      // either a real error with the process's own words, or a confirmed running process.
      const settled = await Promise.race([
        handle.done.then(
          (outcome) => outcome,
          (error) => ({ spawnFailure: String(error?.message ?? error) }),
        ),
        new Promise((resolve) => setTimeout(() => resolve(null), 2500)),
      ])
      if (settled !== null) {
        return {
          ok: false,
          error: settled.spawnFailure
            ? `the bridge process failed: ${settled.spawnFailure}`
            : `the bridge exited immediately (code ${settled.exitCode}, signal ${settled.signal})`,
          argv,
          stdout: readCollected(handle.collected.stdout),
          stderr: readCollected(handle.collected.stderr),
        }
      }

      const health = await call('GET', '/health')
      return {
        ok: true,
        started: true,
        answering: health.reachable,
        argv,
        log: readCollected(handle.collected.stderr),
        note: health.reachable
          ? 'the bridge is up and answering'
          : 'the process is running but is not answering yet; call arm_status in a moment',
      }
    },
  })

  // Tell the model the facts it would otherwise have to rediscover, once per prompt
  // assembly. Bound to this fiber, so removing the plugin removes the section.
  const prompt = ctx.get('systemPrompt')
  if (prompt !== undefined) {
    prompt.section({
      name: 'dsh-so101-arm',
      order: 60,
      text:
        'The DS101 SO-101 arm bench is available through the arm_* tools. Work the loop: ' +
        'arm_status to confirm the bridge and read the safety state, arm_scene to get the ' +
        'object list as numbers, arm_plan before every arm_move_to, then arm_gripper to close ' +
        'and READ possible_grasp — a successful move does not imply a successful grasp. Aim at ' +
        'footprint_centre_base_mm rather than centroid_base_mm. The table sits at negative z in ' +
        'the arm frame. Treat a refused motion as information: the joint envelope, voltage, ' +
        'temperature and per-command step limits exist for a reason, and the software estop is ' +
        'a convenience rather than the real emergency stop.',
    })
  }

  // ── same-origin routes for the browser half ───────────────────────────────────────
  //
  // The panel could have talked to the Python bridge directly, but it must not. The
  // bridge's port is a runtime fact the page has no business knowing, its responses carry
  // no CORS headers, and a second HTTP conversation from the browser would duplicate the
  // reachability and timeout handling that already exists right here. One same-origin hop
  // through the host half keeps the browser half down to rendering JSON and <img> tags —
  // and it is what lets the write surface below be an allow-list instead of a file path
  // the page chooses.
  // The browser half's routes, registered through a SCOPED inject rather than a bare
  // `ctx.get('webServer')`.
  //
  // `ctx.get` reads the registry at this instant. When this plugin's fiber runs before the
  // web carrier has published, it returns undefined and the whole block below is skipped
  // silently — no throw, no warning, no route — and the panel then reports "bridge 离线"
  // forever while the bridge is perfectly healthy. A scoped inject also keeps `webServer`
  // out of this plugin's hard dependency list: a headless deployment loses the panel, not
  // the tools.
  ctx.inject(['webServer'], (scoped) => {
    const home = process.env.DSH_HOME || join(homedir(), '.dsh')
    const deps = { call, workspace, home }
    scoped.effect(
      () =>
        scoped.webServer.register({
          kind: 'prefix',
          // NO trailing slash.
          //
          // The carrier matches a prefix route as `path` plus a path separator, so registering
          // '/so101/' claims '/so101//...' and answers 404 to every real request. The panel then
          // reported "bridge 未运行" -- blaming the one process that was actually healthy --
          // while the route sat on the carrier claiming nothing. better-sidebar registers
          // '/sidebar/file', also with no trailing slash, for the same reason.
          path: ROUTE_PREFIX,
          handler: (req, res) => handleBenchRoute(req, res, deps),
        }),
      'so101.routes',
    )
  })
}

// ── the browser half's same-origin API ──────────────────────────────────────────────

/** The route prefix the browser half talks to. Keep in step with `API` in lib/client.js. */
const ROUTE_PREFIX = '/so101'

/** Ceiling on a configuration file the panel will read or write, in bytes. */
const CONFIG_MAX_BYTES = 2 * 1024 * 1024

/**
 * The only files the browser half may read or write — the panel's entire security boundary.
 *
 * The client sends one of these IDS and never a path, so a buggy or hostile page cannot
 * turn the panel into an arbitrary-file read/write primitive against the host. Exposing
 * another file to the panel means adding a line here, and that is the intended cost.
 */
function configFiles(workspace, home) {
  return [
    {
      id: 'skill',
      label: 'SO-101 operating skill (SKILL.md)',
      path: join(workspace, '.dsh', 'skills', 'dsh-so101-arm', 'SKILL.md'),
    },
    {
      id: 'agents-project',
      label: 'Project instructions (AGENTS.md)',
      path: join(workspace, 'AGENTS.md'),
    },
    {
      id: 'agents-user',
      label: 'User-global instructions (~/.dsh/AGENTS.md)',
      path: join(home, 'AGENTS.md'),
    },
    {
      id: 'calibration',
      label: 'Joint calibration (calib/out/so101_calibration.json)',
      path: join(workspace, 'calib', 'out', 'so101_calibration.json'),
    },
    {
      id: 'runbook',
      label: 'First-run runbook (docs/RUNBOOK)',
      path: join(workspace, 'docs', 'RUNBOOK-首跑手册.md'),
    },
    {
      id: 'plugin-readme',
      label: 'Plugin README',
      path: join(workspace, 'plugins', 'dsh-so101-arm', 'README.md'),
    },
  ]
}

/** Routes forwarded verbatim to the Python bridge, as [method, bridge path]. */
const PROXIED_ROUTES = {
  '/state': ['GET', '/state'],
  '/move_to': ['POST', '/move_to'],
  '/move_joints': ['POST', '/move_joints'],
  '/gripper': ['POST', '/gripper'],
  '/home': ['POST', '/home'],
  '/release': ['POST', '/release'],
  '/estop': ['POST', '/estop'],
  '/enable': ['POST', '/enable'],
}

function sendJson(res, code, value) {
  const body = Buffer.from(JSON.stringify(value), 'utf8')
  res.writeHead(code, {
    'content-type': 'application/json; charset=utf-8',
    'content-length': body.length,
    'cache-control': 'no-store',
  })
  res.end(body)
}

/** Collect a request body, refusing to buffer past the configuration size ceiling. */
function readBody(req) {
  return new Promise((resolve) => {
    const chunks = []
    let size = 0
    req.on('data', (chunk) => {
      size += chunk.length
      if (size <= CONFIG_MAX_BYTES) chunks.push(chunk)
    })
    req.on('end', () => resolve(Buffer.concat(chunks).toString('utf8')))
    req.on('error', () => resolve(''))
  })
}

/**
 * Serve one request from the browser half.
 *
 * Every branch answers with JSON except `/frame`, which streams PNG bytes so the page can
 * use a plain `<img src>`. The handler never rejects: an exception reaching the HTTP server
 * would leave the panel waiting on a request that will never be answered, with nothing in
 * the UI to explain why.
 */
async function handleBenchRoute(req, res, deps) {
  try {
    const url = new URL(req.url ?? '/', 'http://127.0.0.1')
    const route = url.pathname.startsWith(ROUTE_PREFIX)
      ? url.pathname.slice(ROUTE_PREFIX.length).replace(/\/+$/, '') || '/'
      : url.pathname

    if (route === '/status') {
      const health = await deps.call('GET', '/health')
      if (!health.reachable) {
        return sendJson(res, 200, {
          ok: false,
          bridge_reachable: false,
          error: 'bridge unreachable',
          hint: health.hint,
        })
      }
      const state = await deps.call('GET', '/state')
      return sendJson(res, 200, { ok: true, health: health.body, state: state.body })
    }

    if (route === '/frame') {
      const cam = url.searchParams.get('cam') === 'wrist' ? 'wrist' : 'overhead'
      const reply = await deps.call('POST', '/look', { view: cam })
      if (!reply.reachable) {
        return sendJson(res, 503, { ok: false, error: 'bridge unreachable', hint: reply.hint })
      }
      const views = (reply.body && reply.body.views) || {}
      const png = cam === 'wrist' ? views.wrist_png : views.overhead_png
      if (typeof png !== 'string') {
        return sendJson(res, 409, {
          ok: false,
          error: (reply.body && reply.body.error) || `${cam} camera produced no frame`,
        })
      }
      const bytes = readFileSync(png)
      res.writeHead(200, {
        'content-type': 'image/png',
        'content-length': bytes.length,
        'cache-control': 'no-store',
      })
      res.end(bytes)
      return
    }

    if (route === '/files') {
      const files = configFiles(deps.workspace, deps.home).map((f) => {
        let exists = false
        let size = 0
        try {
          const st = statSync(f.path)
          exists = st.isFile()
          size = st.size
        } catch {
          /* a file that is not there yet is a state the panel renders, not an error */
        }
        return { id: f.id, label: f.label, path: f.path, exists, size }
      })
      return sendJson(res, 200, { ok: true, files })
    }

    if (route === '/file/read' || route === '/file/write') {
      let payload
      try {
        payload = JSON.parse(await readBody(req))
      } catch {
        return sendJson(res, 400, { ok: false, error: 'request body is not JSON' })
      }
      const entry = configFiles(deps.workspace, deps.home).find((f) => f.id === payload.id)
      if (entry === undefined) {
        return sendJson(res, 403, { ok: false, error: 'that file id is not on the panel allow-list' })
      }

      if (route === '/file/read') {
        try {
          if (statSync(entry.path).size > CONFIG_MAX_BYTES) {
            return sendJson(res, 413, { ok: false, error: `file is over the ${CONFIG_MAX_BYTES} byte panel limit` })
          }
        } catch {
          return sendJson(res, 404, { ok: false, error: `not found: ${entry.path}` })
        }
        return sendJson(res, 200, {
          ok: true,
          id: entry.id,
          label: entry.label,
          path: entry.path,
          content: readFileSync(entry.path, 'utf8'),
        })
      }

      if (typeof payload.content !== 'string') {
        return sendJson(res, 400, { ok: false, error: 'content must be a string' })
      }
      const bytes = Buffer.byteLength(payload.content, 'utf8')
      if (bytes > CONFIG_MAX_BYTES) {
        return sendJson(res, 413, { ok: false, error: `content is over the ${CONFIG_MAX_BYTES} byte panel limit` })
      }
      writeFileSync(entry.path, payload.content, 'utf8')
      return sendJson(res, 200, { ok: true, id: entry.id, path: entry.path, bytes })
    }

    const target = PROXIED_ROUTES[route]
    if (target !== undefined) {
      let payload = {}
      const raw = await readBody(req)
      if (raw.trim().length > 0) {
        try {
          payload = JSON.parse(raw)
        } catch {
          return sendJson(res, 400, { ok: false, error: 'request body is not JSON' })
        }
      }
      const reply = await deps.call(target[0], target[1], payload)
      if (!reply.reachable) {
        return sendJson(res, 503, { ok: false, error: 'bridge unreachable', hint: reply.hint })
      }
      // The bridge reports a refusal as `ok:false` with an HTTP 4xx; both are information
      // the operator needs, so the status travels through rather than becoming a 200.
      return sendJson(res, 200, { ...reply.body, httpStatus: reply.httpStatus })
    }

    return sendJson(res, 404, { ok: false, error: `no such route: ${route}` })
  } catch (error) {
    if (!res.headersSent) {
      sendJson(res, 500, { ok: false, error: String(error?.message ?? error) })
      return
    }
    try {
      res.end()
    } catch {
      /* the socket is already gone; nothing left to answer */
    }
  }
}
