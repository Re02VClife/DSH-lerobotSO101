/**
 * Load the browser half the way the DSH client module loader does, then actually run it.
 *
 * The client half is the one file in this package no build step touches, which is exactly
 * why it needs a check: a typo here is not caught by any compiler, and its failure mode in
 * the browser is a tab that silently never appears. This harness supplies the two things
 * the loader supplies at runtime — `window.__ModuleLoader__` and `require("react")` — and
 * then goes one step further than the real loader by rendering with an immediate-mode
 * createElement: every function component is invoked on the spot, so a component body that
 * references something undefined throws here instead of blanking the panel.
 *
 * What this deliberately does NOT do is start timers or fetch. `useEffect` is inert, so the
 * polling never runs and nothing touches the bench.
 *
 * Run:  node plugins/dsh-so101-arm/test/client-check.mjs
 */
import { fileURLToPath, pathToFileURL } from 'node:url'
import { dirname, join } from 'node:path'

const here = dirname(fileURLToPath(import.meta.url))
const pkgRoot = join(here, '..')

const failures = []
const check = (label, ok, detail = '') => {
  console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${label}${detail ? ' -> ' + detail : ''}`)
  if (!ok) failures.push(label)
}

// ── the loader's half of the contract ────────────────────────────────────────────────

let bundle = null
globalThis.window = {
  __ModuleLoader__: {
    load(entry) {
      bundle = entry
    },
  },
}

console.log('1. the bundle registers itself with the client module loader')
await import(pathToFileURL(join(pkgRoot, 'lib', 'client.js')).href)
check('window.__ModuleLoader__.load was called', bundle !== null)
check('under the package id', bundle && bundle.id === 'dsh-so101-arm', bundle && bundle.id)
check('with a factory function', bundle && typeof bundle.factory === 'function')

// ── the platform's half: require("react") and an immediate-mode renderer ──────────────

const noop = () => undefined

// The two globals the panel would use to touch the bench. Counting them is what turns "a
// collapsed tab does no work" into an assertion instead of a hope: the effect bodies below
// really run, so a regression that polls while hidden starts a timer or a fetch here.
let timerStarts = 0
/** Every URL the panel asked for, in order. */
const fetched = []
const realSetInterval = globalThis.setInterval
globalThis.setInterval = (...args) => {
  timerStarts += 1
  return realSetInterval(...args)
}
// Never reach the network: a relative URL has no base in Node anyway, and the panel's error
// paths are exactly what these assertions care about.
globalThis.fetch = async (input) => {
  fetched.push(String(input))
  return { ok: false, status: 599, text: async () => 'test stub', blob: async () => new Blob() }
}

let effectRuns = 0
const React = {
  createElement(type, props, ...children) {
    // The real createElement returns a descriptor. This one calls function components
    // immediately, which is what turns the render below into a real execution of every
    // component body in the tree.
    if (typeof type === 'function') return type({ ...(props || {}), children })
    return { type, props: props || {}, children }
  },
  useState(initial) {
    return [typeof initial === 'function' ? initial() : initial, noop]
  },
  useEffect(fn) {
    effectRuns += 1
    // Run the effect body the way a commit would. usePoll's body returns early when
    // `active` is false, so nothing should reach a timer or the network.
    return typeof fn === 'function' ? fn() : undefined
  },
  useRef(value) {
    return { current: value }
  },
  useCallback(fn) {
    return fn
  },
}

const require = (name) => {
  if (name === 'react') return React
  throw new Error(`client bundle required "${name}", which is not a platform seed word`)
}

console.log('\n2. the Cordis plugin export shape')
const exports = bundle.factory(require)
check('exports apply', typeof exports.apply === 'function')
check('exports inject', Array.isArray(exports.inject), JSON.stringify(exports.inject))
check('declares the sidebar service', Array.isArray(exports.inject) && exports.inject.includes('betterSidebar'))

console.log('\n3. a profile without dsh-better-sidebar degrades instead of throwing')
const bare = { get: () => undefined, effect: (fn) => { fn(); return noop } }
const warns = []
const realWarn = console.warn
console.warn = (...args) => warns.push(args.join(' '))
let bareThrew = null
try {
  exports.apply(bare)
} catch (error) {
  bareThrew = error
}
console.warn = realWarn
check('apply() returns quietly when the service is absent', bareThrew === null, bareThrew ? String(bareThrew.message) : '')
check('and says so exactly once, rather than silently doing nothing', warns.length === 1, warns[0] || 'no warning')

console.log('\n4. registration against a recording sidebar service')
const tabs = []
const effects = []
const sidebar = {
  registerTab(descriptor) {
    if (tabs.some((t) => t.id === descriptor.id)) throw new Error(`duplicate tab id: ${descriptor.id}`)
    tabs.push(descriptor)
    return noop
  },
}
const ctx = {
  get(name) {
    return name === 'betterSidebar' ? sidebar : undefined
  },
  effect(callback, label) {
    const disposer = callback()
    effects.push({ label, disposer })
    return noop
  },
}

exports.apply(ctx)

check('two tabs registered', tabs.length === 2, `${tabs.length} tabs`)
check('both registrations are owned by an effect', effects.length === 2,
  effects.map((e) => e.label).join(', '))
check('every registration returned a disposer', effects.every((e) => typeof e.disposer === 'function'))
check('tab ids are namespaced', tabs.every((t) => typeof t.id === 'string' && t.id.startsWith('dsh-so101-arm:')),
  tabs.map((t) => t.id).join(', '))
check('the bench tab is present', tabs.some((t) => t.id === 'dsh-so101-arm:bench'))
check('the config tab is present', tabs.some((t) => t.id === 'dsh-so101-arm:config'))
check('every tab has a title, an order and a component',
  tabs.every((t) => typeof t.title === 'function' && typeof t.order === 'number' && typeof t.component === 'function'))
check('titles resolve to non-empty strings', tabs.every((t) => typeof t.title() === 'string' && t.title().length > 0),
  tabs.map((t) => t.title()).join(' / '))
check('titles are unique', new Set(tabs.map((t) => t.title())).size === tabs.length)

console.log('\n5. every tab renders without throwing')
for (const tab of tabs) {
  let thrown = null
  try {
    // visible:false is the state that must be harmless — no polling, no camera, no fetch.
    const tree = tab.component({ visible: false, tab: { id: tab.id, type: tab.id }, scope: { sessionId: 'test' } })
    if (tree === undefined) thrown = new Error('component returned undefined')
  } catch (error) {
    thrown = error
  }
  check(`${tab.id} renders while hidden`, thrown === null, thrown ? String(thrown.message) : '')
}

// The expensive resource is the RealSense: only one process may hold it, and every frame costs
// real milliseconds. A read of /status is cheap, opens no camera, and the panel makes exactly
// ONE on mount -- without it a hidden tab can say nothing about why it is offline, which is how
// a healthy bridge and a correctly registered route both got reported as "bridge 未运行".
check('a hidden render started no polling timer', timerStarts === 0, `${timerStarts} timers`)
check('a hidden render never opens the camera', fetched.every((u) => !u.includes('/frame')),
  fetched.filter((u) => u.includes('/frame')).join(', ') || 'no frame fetches')
check('a hidden render makes exactly one cheap status probe, not a poll',
  fetched.filter((u) => u.includes('/status')).length === 1,
  fetched.join(', ') || 'nothing fetched')
check('yet the effects ran, so the gate is what stopped the rest', effectRuns > 0, `${effectRuns} effect bodies`)

console.log('\n6. the status formatters accept the payload the bridge actually sends')
// Captured verbatim from a live simulated bridge (`arm_status` → `state`). The point is that
// voltage and temperature are per-joint MAPS, not scalars: the first version of the panel
// assumed scalars and rendered "[object Object] V". A render test cannot catch that, because a
// component rendered once with no data never reaches its data branch — so the formatters are
// exposed as a seam and fed the real shape here.
const fmt = exports.__format
const BRIDGE_STATE = {
  voltage: { shoulder_pan: 4.6, shoulder_lift: 4.6, elbow_flex: 4.6, wrist_flex: 4.6, wrist_roll: 4.6, gripper: 4.6 },
  temperature: { shoulder_pan: 30, shoulder_lift: 30, elbow_flex: 30, wrist_flex: 30, wrist_roll: 30, gripper: 30 },
  tool_mm: [69.4, 9.4, -72.4],
}
check('the formatters are exposed for this check', fmt !== undefined && typeof fmt.span === 'function')
if (fmt !== undefined) {
  check('a per-joint voltage map collapses to one value when every joint agrees',
    fmt.span(BRIDGE_STATE.voltage, 'V') === '4.6 V', String(fmt.span(BRIDGE_STATE.voltage, 'V')))
  check('and becomes a range when they differ',
    fmt.span({ a: 4.5, b: 4.8 }, 'V') === '4.5–4.8 V', String(fmt.span({ a: 4.5, b: 4.8 }, 'V')))
  check('no "[object Object]" can reach the panel',
    !String(fmt.span(BRIDGE_STATE.voltage, 'V')).includes('object'))
  check('the hottest joint is what the temperature line reports',
    fmt.maxText(BRIDGE_STATE.temperature, '°C') === '最高 30 °C', String(fmt.maxText(BRIDGE_STATE.temperature, '°C')))
  check('a bare scalar still works (forward compatibility)',
    fmt.span(4.6, 'V') === '4.6 V', String(fmt.span(4.6, 'V')))
  check('an absent field yields nothing, never the string "undefined"',
    fmt.span(undefined, 'V') === null && fmt.maxText(null, '°C') === null)
}

console.log()
if (failures.length > 0) {
  console.log(`FAILED: ${failures.length} check(s): ${failures.join(' | ')}`)
  process.exit(1)
}
console.log('CLIENT HALF VERIFIED — it loads under the real loader contract, registers cleanly and renders.')
