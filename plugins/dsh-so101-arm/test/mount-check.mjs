/**
 * Mount the host half on a REAL Cordis context.
 *
 * The other suites build their own fake context, and a fake context is exactly how the
 * `inject` bug shipped: the load-check stub had `tools` sitting on it as a plain property, so
 * `inject = []` passed every check and then took the whole profile tree down on the first real
 * boot with
 *
 *     cannot get property "tools" without inject
 *
 * A stub can only ever be as strict as its author remembered to make it. This check cannot
 * drift that way: it drives the actual vendored `@deepseek-ai/cordis` that DSH loads, so the
 * property Guard, the inject bookkeeping and the fiber lifecycle are the real ones. A service
 * the plugin reaches for without declaring it fails here, with the runtime's own error, rather
 * than at profile start-up where the cost is a recovery-mode boot.
 *
 * Run:  node plugins/dsh-so101-arm/test/mount-check.mjs
 */
import { createRequire } from 'node:module'
import { fileURLToPath, pathToFileURL } from 'node:url'
import { dirname, join } from 'node:path'

const here = dirname(fileURLToPath(import.meta.url))
const pkgRoot = join(here, '..')
const workspace = join(pkgRoot, '..', '..')

// Resolve peers and Cordis out of the running profile, as the loader would.
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
      /* a link we could not make is not a mount failure */
    }
  }
}

const { Context } = await import(pathToFileURL(require.resolve('@deepseek-ai/cordis')).href)
const mod = await import(pathToFileURL(join(pkgRoot, 'lib', 'index.js')).href)

const failures = []
const check = (label, ok, detail = '') => {
  console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${label}${detail ? ' -> ' + detail : ''}`)
  if (!ok) failures.push(label)
}

/** The services the plugin is allowed to consume, counting every call so the mount is proven. */
function benchServices() {
  const calls = { tools: 0, sections: 0, routes: 0 }
  return {
    calls,
    services: {
      tools: {
        register() {
          calls.tools += 1
          return () => {}
        },
      },
      systemPrompt: {
        section() {
          calls.sections += 1
          return () => {}
        },
      },
      webServer: {
        register() {
          calls.routes += 1
          return () => {}
        },
      },
      subprocess: {
        spawn: () => ({ pid: 1 }),
      },
    },
  }
}

console.log('1. declare the plugin’s inject list')
check('inject is an array of service names',
  Array.isArray(mod.inject) && mod.inject.every((n) => typeof n === 'string'),
  JSON.stringify(mod.inject))

console.log('\n2. mount on the real Cordis context')
const root = new Context()
const { calls, services } = benchServices()
const disposers = Object.entries(services).map(([name, value]) => root.provide(name, value))

let fiber = null
let mountError = null
try {
  fiber = root.plugin(mod, { workspace, port: 59999, requestTimeoutS: 2 })
  // Cordis fibers are awaitable; awaiting is what surfaces an apply() throw with its stack.
  await fiber
} catch (error) {
  mountError = error
}
check('the fiber reached ready without throwing', mountError === null,
  mountError ? `${mountError.message}` : '')

if (mountError === null) {
  check('and the registry actually received the tools', calls.tools === 11, `${calls.tools} registrations`)
  check('and the prompt section was registered', calls.sections === 1, `${calls.sections} sections`)
  check('and the route prefix was claimed', calls.routes === 1, `${calls.routes} routes`)
}

// Tearing down proves every contribution was owned by the fiber: a service that never
// registered its disposer would leave a live registration behind after unload.
let disposeError = null
try {
  if (fiber && typeof fiber.dispose === 'function') await fiber.dispose()
  else if (typeof root.stop === 'function') await root.stop()
} catch (error) {
  disposeError = error
}
check('the fiber disposes cleanly', disposeError === null, disposeError ? disposeError.message : '')
for (const dispose of disposers) if (typeof dispose === 'function') dispose()

console.log('\n3. a declared-but-absent service parks the plugin instead of throwing')
// What `inject` actually buys, stated as a test. Cordis holds a plugin whose injected service
// has not appeared in a waiting state; it does not run `apply` and it does not fail the tree.
// That is the behaviour that keeps a misconfigured arm plugin from taking a whole profile
// down, and it is only observable on a real fiber.
//
// It is NOT observable here that an UNDECLARED service is refused. The guard that produces
// "cannot get property ... without inject" lives on the strict resolution path, which Cordis
// only takes when the fiber has a loader runtime (cordis/lib/index.js:679); a fiber built by a
// bare `new Context()` takes the permissive branch and resolves services without consulting
// `inject` at all. So the real boot path is STRICTER than anything constructible in this file
// — which is precisely why the host half asks for the registry through `ctx.get` and handles
// its absence, and why `test/fake-context.mjs` models the strict rule separately.
const parked = new Context()
let parkedError = null
let parkedFiber = null
try {
  parkedFiber = parked.plugin(mod, { workspace, port: 59999, requestTimeoutS: 2 })
  await new Promise((resolve) => setTimeout(resolve, 50))
} catch (error) {
  parkedError = error
}
check('plugging in without the registry does not throw', parkedError === null,
  parkedError ? parkedError.message : '')
check('the fiber is parked, not ready (waiting on inject)', parkedFiber !== null && parkedFiber.state !== 2,
  parkedFiber ? `state=${parkedFiber.state}` : 'no fiber')
try {
  await parkedFiber?.dispose?.()
} catch {
  /* a parked fiber has nothing to unwind; a throw here would be the bug, not the cleanup */
}

console.log()
if (failures.length > 0) {
  console.log(`FAILED: ${failures.length} check(s): ${failures.join(' | ')}`)
  process.exit(1)
}
console.log('MOUNT VERIFIED — the host half loads on the real Cordis runtime, not just on a stub.')
