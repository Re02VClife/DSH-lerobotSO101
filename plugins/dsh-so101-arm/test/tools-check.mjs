/**
 * Run every tool's `execute` for real and check what it hands back.
 *
 * `load-check` validates tool SCHEMAS; nothing validated tool RESULTS. That gap shipped a
 * tool which returned `{ pid: undefined }` — `SubprocessHandle` has no `pid` field, only the
 * terminal handle does — and the harness rejected the whole call as "not lossless JSON". The
 * message pointed at the plugin while saying nothing about which field was wrong or that the
 * launch had in fact been attempted.
 *
 * So this suite executes each tool against a bench that is deliberately unreachable, and then
 * walks the result looking for anything JSON cannot carry: `undefined`, functions, symbols,
 * bigints, NaN and Infinity. That is exactly the check the harness applies, done here where it
 * can name the path.
 *
 * Two things are stubbed rather than real, and both matter:
 *   * `fetch` is left alone, so every bridge call fails on a closed port — the deterministic
 *     "bridge unreachable" branch, independent of whether a bench happens to be running;
 *   * `subprocess` is faked with a handle whose process has already exited, so `arm_launch`
 *     exercises its failure path and never actually starts a Python process from a test.
 *
 * Run:  node plugins/dsh-so101-arm/test/tools-check.mjs
 */
import { createRequire } from 'node:module'
import { fileURLToPath, pathToFileURL } from 'node:url'
import { dirname, join } from 'node:path'
import { guardContext } from './fake-context.mjs'

const here = dirname(fileURLToPath(import.meta.url))
const pkgRoot = join(here, '..')
const workspace = join(pkgRoot, '..', '..')

const profileModules = 'C:\\Users\\XU\\.dsh\\profiles\\node_modules'
const require = createRequire(join(profileModules, 'noop.js'))
const { mkdirSync, symlinkSync, existsSync } = await import('node:fs')
const shimDir = join(pkgRoot, 'node_modules', '@deepseek-ai')
mkdirSync(shimDir, { recursive: true })
for (const dep of ['dsh-tools', 'schemastery']) {
  const target = join(profileModules, '@deepseek-ai', dep)
  const link = join(shimDir, dep)
  if (existsSync(target) && !existsSync(link)) {
    try {
      symlinkSync(target, link, 'junction')
    } catch {
      /* a link we could not make is not a result-shape failure */
    }
  }
}

const mod = await import(pathToFileURL(join(pkgRoot, 'lib', 'index.js')).href)

const failures = []
const check = (label, ok, detail = '') => {
  console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${label}${detail ? ' -> ' + detail : ''}`)
  if (!ok) failures.push(label)
}

/**
 * The harness's own rule, applied here: find the first value JSON cannot carry.
 *
 * `JSON.stringify` is not enough on its own — it silently DROPS an `undefined` object value
 * and turns `NaN` into `null` — so a round-trip comparison would pass on exactly the payload
 * that was rejected.
 */
function firstNonJsonValue(value, path = '$') {
  if (value === null) return null
  const type = typeof value
  if (type === 'string' || type === 'boolean') return null
  if (type === 'number') return Number.isFinite(value) ? null : `${path} = ${String(value)}`
  if (type === 'undefined') return `${path} = undefined`
  if (type !== 'object') return `${path} = ${type}`
  if (Array.isArray(value)) {
    for (let i = 0; i < value.length; i += 1) {
      const bad = firstNonJsonValue(value[i], `${path}[${i}]`)
      if (bad !== null) return bad
    }
    return null
  }
  for (const key of Object.keys(value)) {
    const bad = firstNonJsonValue(value[key], `${path}.${key}`)
    if (bad !== null) return bad
  }
  return null
}

/** A subprocess handle whose child has already exited, carrying a reason on stderr. */
function deadHandle(stderrText) {
  const reader = (text) => ({
    readFrom: () => ({ text, nextOffset: text.length, lossy: false }),
  })
  return {
    stdin: undefined,
    stdout: undefined,
    stderr: undefined,
    collected: {
      stdout: reader(''),
      stderr: reader(stderrText),
    },
    done: Promise.resolve({ exitCode: 1, signal: null }),
    terminate() {},
    waitForExit: async () => true,
  }
}

const definitions = []
const ctx = guardContext({
  inject: mod.inject,
  services: {
    tools: {
      register(definition) {
        definitions.push(definition)
        return () => {}
      },
    },
    subprocess: {
      spawn: () =>
        deadHandle('ModuleNotFoundError: No module named \'pyrealsense2\''),
    },
  },
})
mod.apply(ctx, { workspace, port: 59999, requestTimeoutS: 2 })

console.log('1. every tool is callable')
const ARGS = {
  arm_status: {},
  arm_scene: {},
  arm_pixel_to_base: { u: 320, v: 240 },
  arm_plan: { target_mm: [200, 0, 100] },
  arm_move_to: { target_mm: [200, 0, 100] },
  arm_gripper: { openness_pct: 50 },
  arm_home: {},
  arm_estop: {},
  arm_enable: {},
  arm_release: {},
  arm_launch: { simulate: true },
}
check('all eleven tools registered', definitions.length === 11, `${definitions.length}`)
check('every tool has an execute function',
  definitions.every((d) => typeof d.execute === 'function'),
  definitions.filter((d) => typeof d.execute !== 'function').map((d) => d.name).join(', ') || 'all have one')

for (const definition of definitions) {
  const args = ARGS[definition.name]
  if (args === undefined) {
    check(`${definition.name} has args in this suite`, false, 'no ARGS entry — add one')
    continue
  }

  let result = null
  let thrown = null
  try {
    result = await definition.execute(args)
  } catch (error) {
    thrown = error
  }

  if (thrown !== null) {
    check(`${definition.name} executes without throwing`, false, String(thrown.message || thrown))
    continue
  }

  const bad = firstNonJsonValue(result)
  check(`${definition.name} returns lossless JSON`, bad === null, bad || 'clean')

  // A tool that answers a broken bench with silence is worse than one that fails loudly, so
  // every unreachable-bridge reply must also carry something an operator can act on.
  if (definition.name !== 'arm_launch') {
    check(`${definition.name} reports the unreachable bridge`,
      result !== null && typeof result === 'object' && result.bridge_reachable === false,
      JSON.stringify(result && result.error))
  }
}

console.log('\n2. arm_launch reports WHY a bridge failed to start')
const launch = definitions.find((d) => d.name === 'arm_launch')
const launched = await launch.execute({ simulate: true })
check('a process that exits immediately is reported as a failure', launched.ok === false, JSON.stringify(launched.error))
check('and the process\'s own stderr is carried back', typeof launched.stderr === 'string' && launched.stderr.length > 0,
  JSON.stringify(launched.stderr))
check('the failed launch is still lossless JSON', firstNonJsonValue(launched) === null,
  firstNonJsonValue(launched) || 'clean')

console.log()
if (failures.length > 0) {
  console.log(`FAILED: ${failures.length} check(s): ${failures.join(' | ')}`)
  process.exit(1)
}
console.log('TOOL RESULTS VERIFIED — every execute() runs, and every result survives the JSON boundary.')
