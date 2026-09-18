/**
 * Load-check the plugin the way the loader will: import it for real, then call apply()
 * against the genuine @deepseek-ai modules with a recording fake context.
 *
 * This is the check that catches the failure modes a text review cannot:
 *   * a bad import or a named export that does not exist;
 *   * a `parameters` or `output` shape defineTool rejects;
 *   * a typo in a tool name or a duplicate registration;
 *   * `ctx.systemPrompt.section()` being called with a shape the registry refuses.
 *
 * Run:  node plugins/dsh-so101-arm/test/load-check.mjs
 */
import { createRequire } from 'node:module'
import { fileURLToPath, pathToFileURL } from 'node:url'
import { dirname, join } from 'node:path'
import { guardContext } from './fake-context.mjs'

const here = dirname(fileURLToPath(import.meta.url))
const pkgRoot = join(here, '..')

// resolve the harness packages out of the running profile
const profileModules = 'C:\\Users\\XU\\.dsh\\profiles\\node_modules'
const require = createRequire(join(profileModules, 'noop.js'))

const failures = []
const check = (label, ok, detail = '') => {
  console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${label}${detail ? ' -> ' + detail : ''}`)
  if (!ok) failures.push(label)
}

// The plugin imports its peers bare. Node resolves bare specifiers by walking up from
// the importing file, and this package sits outside the profile, so bridge the gap the
// same way an install would: a scoped symlink in a local node_modules.
const shimDir = join(pkgRoot, 'node_modules', '@deepseek-ai')
const { mkdirSync, symlinkSync, existsSync } = await import('node:fs')
mkdirSync(shimDir, { recursive: true })
for (const dep of ['dsh-tools', 'schemastery']) {
  const target = join(profileModules, '@deepseek-ai', dep)
  const link = join(shimDir, dep)
  if (!existsSync(target)) {
    console.log(`  (note: ${target} not present; skipping the link)`)
    continue
  }
  if (!existsSync(link)) {
    try {
      symlinkSync(target, link, 'junction')
    } catch (error) {
      console.log(`  (could not link ${dep}: ${error.message})`)
    }
  }
}

console.log('1. module contract')
const mod = await import(pathToFileURL(join(pkgRoot, 'lib', 'index.js')).href)
check('imports without error', true)
check('exports name', typeof mod.name === 'string', mod.name)
check('exports apply', typeof mod.apply === 'function')
check('exports inject as an array', Array.isArray(mod.inject), JSON.stringify(mod.inject))

console.log('\n2. apply() against a recording context')
const registered = []
const sections = []
const sink = { provided: null, effects: [] }

// The stub is built through guardContext so that `ctx.tools` is reachable ONLY because the
// plugin declares `inject = ['tools']`. A permissive stub here is what let an earlier version
// ship with `inject = []` and crash the whole profile tree at boot.
const ctx = guardContext({
  inject: mod.inject,
  sink,
  services: {
    tools: {
      register(definition) {
        if (registered.some((d) => d.name === definition.name)) {
          throw new Error(`duplicate tool name: ${definition.name}`)
        }
        registered.push(definition)
        return () => {}
      },
    },
    systemPrompt: {
      section(section) {
        sections.push(section)
        return () => {}
      },
    },
    subprocess: {
      spawn: (spec) => ({ pid: 12345, argv: spec.argv }),
    },
  },
})

check('the plugin declares the tools registry it uses', mod.inject.includes('tools'),
  JSON.stringify(mod.inject))

let applyError = null
try {
  mod.apply(ctx, { workspace: pkgRoot, python: 'python', port: 8731, allowMotion: false })
} catch (error) {
  applyError = error
}
check('apply() did not throw', applyError === null, applyError ? applyError.message : '')
check('registered no service (this row binds nothing)', sink.provided === null)

const expected = [
  'arm_status', 'arm_scene', 'arm_pixel_to_base', 'arm_plan', 'arm_move_to',
  'arm_gripper', 'arm_home', 'arm_estop', 'arm_enable', 'arm_release', 'arm_launch',
]
const names = registered.map((d) => d.name).sort()
check('registered every expected tool', expected.every((n) => names.includes(n)),
  `${names.length} tools: ${names.join(', ')}`)
check('registered no unexpected tool', names.every((n) => expected.includes(n)),
  names.filter((n) => !expected.includes(n)).join(', ') || 'none')

console.log('\n3. every RAW definition the plugin builds passes the real defineTool')
// Re-validate the options this module actually hands to defineTool. Feeding back the RETURN
// value would be meaningless: defineTool normalizes `parameters` into a compiled JSON schema,
// so validating that checks the harness rather than the plugin. `__rawToolDefinitions` exposes
// the input side for exactly this purpose.
const { defineTool } = require('@deepseek-ai/dsh-tools')
const raw = mod.__rawToolDefinitions
check('the plugin exposed its raw definitions for this check', Array.isArray(raw) && raw.length > 0,
  `${raw?.length ?? 0} definitions`)
let validated = 0
for (const options of raw) {
  let ok = true
  let detail = ''
  try {
    const normalized = defineTool(options)
    ok = normalized.name === options.name
  } catch (error) {
    ok = false
    detail = error.message
  }
  if (ok) validated += 1
  check(`${options.name} is a valid definition`, ok, detail)
  check(`${options.name} has a model-usable description`,
    typeof options.description === 'string' && options.description.length >= 60,
    `${options.description?.length ?? 0} chars`)
  check(`${options.name} declares a parameters map (not a schema root)`,
    options.parameters !== null && typeof options.parameters === 'object'
      && Array.isArray(options.parameters) === false,
    Array.isArray(options.parameters) ? 'array' : typeof options.parameters)
  check(`${options.name} output declares an explicit object root`,
    options.output?.schema?.type === 'object'
      && typeof options.output.schema.additionalProperties === 'boolean',
    JSON.stringify(options.output?.schema ?? null))
}
check('all raw definitions validated', validated === raw.length, `${validated}/${raw.length}`)

console.log('\n4. required parameters are declared required')
const needRequired = {
  arm_pixel_to_base: ['u', 'v'],
  arm_plan: ['target_mm'],
  arm_move_to: ['target_mm'],
  arm_gripper: ['openness_pct'],
}
for (const [toolName, keys] of Object.entries(needRequired)) {
  const options = raw.find((x) => x.name === toolName)
  if (!options) { check(`${toolName} present`, false); continue }
  for (const key of keys) {
    check(`${toolName}.${key} is required`, options.parameters?.[key]?.required === true,
      JSON.stringify(options.parameters?.[key] ?? null))
  }
}

console.log('\n5. prompt section')
check('registered exactly one prompt section', sections.length === 1, String(sections.length))
if (sections[0]) {
  check('section has a namespaced name', /^[\w.:-]+$/.test(sections[0].name ?? ''), sections[0].name)
  check('section has a numeric order', typeof sections[0].order === 'number', String(sections[0].order))
  check('section text is substantive',
    typeof sections[0].text === 'string' && sections[0].text.length > 200,
    `${sections[0].text?.length ?? 0} chars`)
}

console.log('\n6. an absent registry disables the plugin instead of throwing')
// The failure mode this plugin is built around: no registry means no tools, never a crash.
// On a real profile `inject: ['tools']` means it cannot happen — but if a scope change or the
// strict guard ever produces it anyway, the profile has to survive, so the registry is read
// through ctx.get rather than assumed.
const noRegistry = guardContext({ inject: mod.inject, services: {} })
const realWarn = console.warn
const warnCalls = []
console.warn = (...args) => warnCalls.push(args.join(' '))
let noRegistryThrew = null
try {
  mod.apply(noRegistry, { workspace: pkgRoot, python: 'python', port: 8731 })
} catch (error) {
  noRegistryThrew = error
}
console.warn = realWarn
check('apply() returns instead of throwing', noRegistryThrew === null,
  noRegistryThrew ? noRegistryThrew.message : '')
check('and says why, once', warnCalls.length === 1, warnCalls[0] || 'no warning')

console.log('\n7. the guard itself works')
// A test harness that cannot fail is not a test. This proves guardContext actually refuses an
// undeclared service, which is the exact check that would have turned the boot crash into a
// red line here instead of a profile that will not start.
let guardFired = null
try {
  const strict = guardContext({ inject: [], services: { tools: { register() {} } } })
  strict.tools.register({ name: 'never-registered' })
} catch (error) {
  guardFired = error
}
check('touching an undeclared service throws', guardFired !== null)
check('with the runtime wording',
  guardFired !== null && /cannot get property "tools" without inject/.test(guardFired.message),
  guardFired ? guardFired.message : 'no error')

console.log()
if (failures.length) {
  console.log(`FAILED: ${failures.length} check(s): ${failures.join('; ')}`)
  process.exit(1)
}
console.log('PLUGIN LOADS — module contract, all tool definitions and the prompt section are valid.')
