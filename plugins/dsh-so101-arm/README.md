# dsh-so101-arm

Give a [DSH](https://github.com/deepseek-ai/deepseek-harness) agent hands on a **LeRobot
SO-101** arm.

Ten tools: structured bench perception, IK dry runs, rate-limited motion, grasp feedback,
and a software stop. The model never touches a servo — it asks for a Cartesian target, and
a local bridge that owns the hardware decides whether that is safe.

For the human at the bench there is also a **right-column console**: both camera feeds, the
joint angles on sliders, and the bench's own configuration files, contributed to the DSH
right sidebar through `dsh-better-sidebar`. See
[The right-column console](#the-right-column-console).

```
agent ──arm_* tools──▶ this plugin ──HTTP/loopback──▶ bridge (Python) ──COM10──▶ SO-101
browser ──/so101/*───▶      │                              │
                            └──────────────────────────────┴──USB──▶ overhead RealSense + wrist cam
```

## Why a bridge, and not direct control

A tool call crosses a process boundary and an agent round-trip is seconds. A stalled servo
is milliseconds. So the split is a design rule rather than a convenience:

| Concern | Owner |
|---|---|
| Serial bus, cameras, calibration file | the **bridge** — one process, one hardware owner |
| Joint envelope, rate limiter, voltage/temperature pre-flight, heartbeat watchdog | the **bridge** |
| Tool schemas, defaults, turning a refusal into something an agent can act on | **this plugin** |

The plugin holds no hardware handle, so unloading it can never leave a COM port locked.

## Install

This package is a **Host-plane profile bundle**: composing it gives every session the arm.

1. Add it to a profile and install it. From the profile directory
   (`$DSH_HOME/profiles/<name>/`):

   ```sh
   pnpm add dsh-so101-arm          # or:  pnpm add file:/path/to/dsh-so101-arm
   ```

   and list it in that profile's `package.json`:

   ```json
   {
     "dsh": {
       "profile": {
         "bundles": [
           "@deepseek-ai/dsh-base",
           "@deepseek-ai/dsh-web-app",
           "dsh-so101-arm"
         ]
       }
     }
   }
   ```

   The package ships `cordis.patch.yml`, which inserts exactly one row:

   ```yaml
   - insert:
       - id: so101-arm
         name: 'dsh-so101-arm'
   ```

   For the right-column console, install the sidebar service in the same profile. It is
   what publishes `ctx.betterSidebar`, the only supported way to contribute a tab to the
   DSH right column:

   ```sh
   dsh plugin --profile <name> add dsh-better-sidebar@latest
   # pnpm 11 blocks node-pty's build script on the first run; approve it and re-run:
   #   cd $DSH_HOME/profiles/<name> && pnpm approve-builds --all
   #   dsh plugin --profile <name> add dsh-better-sidebar@latest
   ```

   This step is optional — without it the plugin loads and the `arm_*` tools work; only the
   panel is missing.

2. **Restart the profile.** Composition is read at start-up, so a running session will not
   pick the row up.

3. Configure it under `dsh-so101-arm:` in the profile's `settings.yaml`:

   ```yaml
   dsh-so101-arm:
     workspace: D:\vscode\DS101
     python: D:\vscode\DS101\.venv\Scripts\python.exe
     port: 8731
     serialPort: COM10
     allowMotion: false        # set true only when you are ready for the arm to move
     tableZMm: -115            # the shoulder axis, in mm above the table
     cameraForwardMm: 440
     cameraHeightMm: 625
     cameraPitchDeg: 16
   ```

4. Start a session and confirm the tool list contains `arm_status`, `arm_plan`,
   `arm_move_to`, `arm_gripper`, `arm_estop`.

### `allowMotion` is off by default, deliberately

Installing a plugin must never be able to energise a robot. With `allowMotion: false` the
bridge starts read-only: every move is refused, and every read still works. Turn it on when
you have the arm on a bench, the workspace clear, and a physical emergency stop wired to
the servo power.

## Composed on the host plane, or in a preset?

Either works — the row publishes no service, so it needs no `isolate` realm, which is the
rule that normally decides this. The default in `cordis.patch.yml` is the **host** plane
because a robot bench is one physical resource driven by one operator, and a per-session
copy would let two sessions believe they own the same arm.

If you would rather only some agents have hands, drop the row into an agent preset's
`agent.cordis.yml` instead of the profile bundle.

## The right-column console

`dsh-better-sidebar` is an **optional** peer. When it is mounted, this package contributes
two tabs to the DSH right column:

| Tab | What it does |
|---|---|
| **SO-101 机械臂** | both camera feeds, live status (voltage, temperature, tool point), one slider per joint with an explicit *apply*, home / open / close, and an operation log |
| **SO-101 配置** | reads and writes the bench's own configuration files — the operating skill, `AGENTS.md`, the joint calibration, the runbook |

Without that package the plugin still loads and the `arm_*` tools are unaffected; the panel
is the only thing that goes missing, and the client half says so once in the console rather
than failing silently.

### How the browser half reaches the bench

It does not talk to the Python bridge. The host half registers a same-origin route prefix,
`/so101/`, and the browser half only ever talks to that:

| Route | Returns |
|---|---|
| `GET /so101/status` | bridge health plus the joint state |
| `GET /so101/frame?cam=overhead\|wrist` | a PNG, so the panel can use a plain `<img src>` |
| `GET /so101/files` | the editable-file inventory |
| `POST /so101/file/read`, `POST /so101/file/write` | one file's contents |
| `POST /so101/{state,move_to,move_joints,gripper,home,release,estop,enable}` | forwarded to the bridge |

Two properties are load-bearing:

- **Same origin** means no CORS negotiation, no loopback port for the browser to discover,
  and no credentials crossing a boundary the page does not control.
- **The file surface is an allow-list keyed by id, never by path.** The page names an id and
  the host half resolves it, so a buggy or hostile page cannot turn the panel into an
  arbitrary-file read/write primitive against the host. Widening the panel means editing
  `configFiles()` in `lib/index.js` — that is the intended cost.

Motion still passes the bridge's own `--allow-motion` gate, so the panel cannot move the arm
unless the bridge was started willing to move it, and it can never exceed the joint envelope,
step limits or watchdog the bridge already enforces. The panel's stop is the same software
stop as `arm_estop` — it is **not** the physical emergency stop.

## Prerequisites

- **The bridge.** This plugin drives an HTTP service that lives in the bench repository
  (`bridge/server.py`). It is not bundled here, because it is hardware-specific: it needs
  the SO-101 URDF, the calibration file for *your* arm, and the camera serial.
- **A calibration.** Every pose number is meaningless without a joint-zero calibration for
  that specific arm. The bridge refuses to start without one.
- **Python** with `pyrealsense2`, `opencv-python`, `numpy`, `pyserial` and `scservo_sdk`.
- **A physical emergency stop.** The software `arm_estop` releases torque; it is not a
  substitute for cutting servo power.

## Tools

| Tool | Purpose |
|---|---|
| `arm_status` | joint angles, tool position, voltage, temperature, bridge reachability, motion permission |
| `arm_scene` | **structured** perception: table plane plus per-object centroid, extent, height, colour and footprint centre, in mm |
| `arm_pixel_to_base` | pixel + depth → arm-frame millimetres, via the hand-eye calibration |
| `arm_plan` | **dry run**: IK solution, residual and reachability, without moving |
| `arm_move_to` | Cartesian move through IK, the rate limiter and the joint envelope |
| `arm_gripper` | open/close, and report `possible_grasp` |
| `arm_estop` / `arm_enable` / `arm_release` | software stop, re-enable after pre-flight, release torque for manual positioning |
| `arm_launch` | start the bridge in the background (refuses if one is already serving) |

### Read `status`, not `ok`

`ok: true` means the request was accepted. `status` says what happened:

- `reached` — the joints arrived within tolerance;
- `clamped` — the target was outside the joint envelope, so the arm moved as far as allowed
  and **did not arrive**;
- `budget_exhausted` — the per-command rate limiter needs another call; nothing is wrong.

### `possible_grasp` is the only grasp feedback

These servos have no force sensing. A jaw that closes on nothing travels its full range; a
jaw that closes on an object stalls early. **A successful move does not imply a successful
grasp** — read the flag.

## A skill ships with it

`skills/dsh-so101-arm/SKILL.md` is the operating knowledge: the mount geometry, the reach
limits, what each status means, and the failure modes that look like code bugs but are
physics. Copy it into a skill root (for example `<project>/.dsh/skills/`) or point a
filesystem skill provider at the package. It is deliberately separate from the tool
descriptions: descriptions teach the *call*, the skill teaches the *job*.

## Develop

```sh
node test/load-check.mjs      # host half: module contract, tool schemas, prompt section
node test/routes-check.mjs    # host half: the /so101/ routes and the file allow-list
node test/client-check.mjs    # client half: loader contract, registration, render
node test/mount-check.mjs     # host half: mounts on the real vendored Cordis
node test/tools-check.mjs     # host half: every execute() runs, every result is lossless JSON
```

All three import the package for real rather than re-reading text. `load-check` calls
`apply()` against a recording context using the genuine `@deepseek-ai/dsh-tools`, so every
tool definition is validated by the same code the loader uses. `routes-check` drives the
registered HTTP handler with synthetic requests, which is where the file allow-list is
actually proven — a regression that accepts a path instead of an id fails there.
`client-check` supplies the two things the browser loader supplies (`window.__ModuleLoader__`
and `require("react")`) and then renders immediately, invoking every component body, so a
reference to something undefined throws in the test instead of blanking the panel.
`tools-check` goes one step further than schema validation and actually *executes* every tool
against an unreachable bench, then walks each result for anything JSON cannot carry.

#### Schemas passing is not results passing

Two bugs came from the same blind spot: code that had never once run.

`arm_launch` returned `{ pid: handle.pid }`, but `SubprocessHandle` has **no** `pid` field —
only `SubprocessTerminalHandle` does. So `pid` was `undefined`, the harness rejected the whole
call as "not lossless JSON", and the error pointed at the plugin without naming the field or
admitting that the launch had been attempted at all.

The panel rendered `state.voltage` as a scalar. It is a per-joint **map** (`{ joint: value }`,
see `state()` in `bridge/driver.py`), so the line read `[object Object] V`. The render test
missed it because it only ever rendered with `status === null`: a component rendered once with
no data never reaches its data branch, and rendering once is not the same as rendering with
something to render.

`tools-check` now covers the first case directly (`JSON.stringify` is not enough — it silently
drops `undefined` and turns `NaN` into `null`, so the check walks the value instead), and
`client-check` covers the second by feeding the formatters a payload captured from a live
bridge.

They link their peer dependencies out of the running profile, so run them on a machine that
has DSH installed. That makes them machine-bound by construction — a plugin that resolves its
peers from a profile cannot be checked without one.

### The `inject` trap this package already fell into

`ctx.<service>` naming a service that is NOT in `inject` is refused by the Cordis property
guard. The first version of this package declared `inject = []` and called
`ctx.tools.register(...)`, which fails at load with:

```
cannot get property "tools" without inject
```

That failure does not degrade one plugin. The throw propagates out through the profile's
`include` row, and one throwing row takes the whole tree down — DSH comes up in **recovery
mode** because of a robot plugin.

Two independent things were wrong, and both are now fixed.

**The plugin could take the profile down.** It now asks for the registry with
`ctx.get('tools')` and returns with a warning when it is absent, while still declaring
`inject: ['tools']`. The two are not redundant: `inject` is a promise about *ordering* (the
plugin activates once the service exists), `ctx.get` is a promise about the *failure mode* (a
missing service disables the panel instead of throwing). A plugin should never be able to do
more damage than not working.

**The tests were more permissive than the runtime.** `load-check` handed `apply()` a stub with
`tools` sitting on it as a plain property, so `inject = []` passed every check and then crashed
on the first real boot. That is worse than having no test, because it converts a
crash-on-startup into a green check. `test/fake-context.mjs` now builds the context through a
`Proxy` that enforces the strict rule, and it reads `inject` off the module under test so the
stub cannot drift from what the plugin declares.

#### Why the strict stub has to be hand-made

Cordis only enforces the guard on its strict resolution path, and it only takes that path when
the fiber has a loader runtime:

```js
// cordis/lib/index.js:679
if (!ctx.fiber.runtime) return ctx.reflect.get(prop, false);   // permissive: no inject check
return ctx.events.waterfall("internal/get", ...)               // strict: isolate + inject
```

A fiber built by a bare `new Context()` has no runtime, so `test/mount-check.mjs` — which does
mount the plugin on the real vendored Cordis — still cannot reproduce the refusal. **The real
boot path is stricter than anything constructible in-process.** That is the whole reason the
strict fake context exists rather than being replaced by a "real" one.

`mount-check` earns its place differently: it proves the module mounts at all, that every
contribution is owned by the fiber and disposed with it, and that a declared-but-absent service
*parks* the plugin (`state=0`, `apply` never runs) instead of throwing — which is the behaviour
that makes a misconfiguration survivable.

The rule to carry forward:

| Access | Use |
|---|---|
| a service the plugin cannot work without | `inject: ['name']`, then `ctx.get('name')` with a check |
| a service it can live without | `ctx.get('name')` plus an `undefined` check |
| a bare `ctx.<name>` | only after declaring it, and prefer `get` anyway |

#### Before restarting a profile

A changed host half only takes effect on a profile restart, and there is no in-process
substitute for a real boot. The closest thing is to run all four suites, then confirm the row
reached the composed tree:

```sh
node test/load-check.mjs && node test/routes-check.mjs && \
node test/client-check.mjs && node test/mount-check.mjs

dsh --profile <name> --dump-config | grep -A2 so101-arm
```

`--dump-config` prints the composed tree without running any plugin, so it confirms the mount
relationship but **not** that `apply()` succeeds — do not read a clean dump as a clean boot.

It is worth running after any edit: the tool schema DSL has a trap that no amount of reading
catches. `parameters` is a **map of parameter name to property spec**, compiled by the
registry into an implicit open object root — so it must not carry a `type` key. Adding one
fails with `parameters.type must be a value schema object`, which reads like the opposite of
the real problem.

## License

MIT
