/**
 * Exercise the browser half's same-origin routes for real: import the host half, call
 * apply() against a recording context, then drive the registered HTTP handler with
 * synthetic requests.
 *
 * What this catches that a text review cannot:
 *   * the allow-list actually refusing an id that is not on it — the panel's whole
 *     security story is that the page names an ID and never a path, and a regression that
 *     accepts a path turns the plugin into an arbitrary-file read/write primitive;
 *   * a route that throws instead of answering, which in the browser is an `<img>` that
 *     never loads and a panel with no explanation;
 *   * a body that is not JSON, and a bridge that is not running, both answering with a
 *     status the panel can render.
 *
 * The bridge is deliberately pointed at a closed port, so the "unreachable" branches are
 * the ones under test and the result does not depend on whether the bench is up.
 *
 * Run:  node plugins/dsh-so101-arm/test/routes-check.mjs
 */
import { createRequire } from 'node:module'
import { fileURLToPath, pathToFileURL } from 'node:url'
import { dirname, join } from 'node:path'
import { Readable } from 'node:stream'
import { guardContext } from './fake-context.mjs'

const here = dirname(fileURLToPath(import.meta.url))
const pkgRoot = join(here, '..')
const workspace = join(pkgRoot, '..', '..')

// Resolve the plugin's peers out of the running profile, exactly as load-check.mjs does.
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
      /* a link we could not make is not a routing failure */
    }
  }
}

const failures = []
const check = (label, ok, detail = '') => {
  console.log(`  [${ok ? 'PASS' : 'FAIL'}] ${label}${detail ? ' -> ' + detail : ''}`)
  if (!ok) failures.push(label)
}

const mod = await import(pathToFileURL(join(pkgRoot, 'lib', 'index.js')).href)

console.log('1. mount the routes')
let route = null
const ctx = guardContext({
  inject: mod.inject,
  services: {
    tools: { register: () => () => {} },
    webServer: {
      register(entry) {
        route = entry
        return () => {}
      },
    },
  },
})
// A closed port, so every bridge call in this run takes the unreachable branch.
mod.apply(ctx, { workspace, port: 59999, requestTimeoutS: 2 })

check('webServer.register was called', route !== null)
// The carrier's own prefix rule, copied verbatim from dsh-host-webserver's matcher:
//     if (pathname !== prefix && !pathname.startsWith(`${prefix}/`)) continue
// Encoding it here is the entire point. This suite used to assert the literal string
// '/so101/', so it stayed green while the route claimed '/so101//...' and EVERY real request
// fell through to the fallback and answered 404 -- which the panel reported as "bridge 未运行",
// blaming the one process that was healthy. Assert the CLAIM, never the spelling.
const carrierPrefixMatches = (pathname, prefix) =>
  pathname === prefix || pathname.startsWith(`${prefix}/`)

check('registered as a prefix route', route && route.kind === 'prefix', route && route.kind)
check('the prefix claims the path the panel actually calls',
  route && route.kind === 'prefix' && carrierPrefixMatches('/so101/status', route.path),
  route ? `path=${route.path} vs /so101/status` : 'nothing registered')
check('and claims the camera and file routes the panel also uses',
  route && ['/so101/frame', '/so101/files', '/so101/file/read', '/so101/file/write']
    .every((p) => carrierPrefixMatches(p, route.path)),
  route ? `path=${route.path}` : 'nothing registered')
check('handler is a function', route && typeof route.handler === 'function')

/** One synthetic request/response pair. */
function makeRes() {
  return {
    statusCode: null,
    headers: null,
    body: null,
    headersSent: false,
    writeHead(code, headers) {
      this.statusCode = code
      this.headers = headers
      this.headersSent = true
    },
    end(body) {
      this.body = body === undefined ? null : body
    },
  }
}

function makeReq(method, url, body, raw) {
  // `raw` bypasses JSON.stringify so a test can send bytes that are genuinely not JSON.
  // Without it a "malformed body" case is stringify'd into a valid JSON string literal and
  // the route under test never sees malformed input at all.
  const payload =
    body === undefined ? undefined : raw === true ? String(body) : JSON.stringify(body)
  const chunks = payload === undefined ? [] : [Buffer.from(payload, 'utf8')]
  const req = Readable.from(chunks)
  req.method = method
  req.url = url
  return req
}

async function call(method, url, body, raw) {
  const req = makeReq(method, url, body, raw)
  const res = makeRes()
  await route.handler(req, res)
  let json = null
  if (res.body !== null) {
    try {
      json = JSON.parse(res.body.toString('utf8'))
    } catch {
      json = null
    }
  }
  return { status: res.statusCode, headers: res.headers, json, bytes: res.body }
}

console.log('\n2. the configuration allow-list')
const files = await call('GET', '/so101/files')
check('GET /so101/files answers 200', files.status === 200, `HTTP ${files.status}`)
check('and lists every entry', files.json && Array.isArray(files.json.files) && files.json.files.length === 6,
  files.json && files.json.files ? `${files.json.files.length} files` : 'no list')
check('each entry carries id, label and path',
  !!(files.json && files.json.files.every((f) => f.id && f.label && f.path)))
check('the skill file exists on this bench',
  !!(files.json && files.json.files.find((f) => f.id === 'skill' && f.exists)))

console.log('\n3. read is bounded by that list')
const skill = await call('POST', '/so101/file/read', { id: 'skill' })
check('reading a listed file answers 200', skill.status === 200, `HTTP ${skill.status}`)
check('and returns its text', !!(skill.json && typeof skill.json.content === 'string' && skill.json.content.length > 100),
  skill.json ? `${(skill.json.content || '').length} chars` : 'no content')

// The panel reads the calibration through this very route to get its slider bounds and its HOME
// default. If this id stops resolving, the panel silently falls back to a symmetric +/-180 range
// and seeds the sliders from the live pose -- which is how "the sliders open on home" quietly
// became "the sliders show wherever the arm happens to be". A fallback that nobody notices is
// the same failure mode as the trailing-slash route: visible only on the bench.
const calibRead = await call('POST', '/so101/file/read', { id: 'calibration' })
check('the calibration is readable through the panel route',
  calibRead.status === 200 && !!(calibRead.json && typeof calibRead.json.content === 'string'),
  `HTTP ${calibRead.status}`)
let parsedCalib = null
try {
  parsedCalib = JSON.parse(calibRead.json.content)
} catch {
  /* reported by the check below */
}
check('and carries joint_limits_deg + home_deg, which the sliders depend on',
  !!(parsedCalib && parsedCalib.joint_limits_deg && parsedCalib.home_deg),
  parsedCalib ? Object.keys(parsedCalib).join(',') : 'unparsable')

const traversal = await call('POST', '/so101/file/read', { id: '../../../../etc/passwd' })
check('an id that is not on the list is refused with 403', traversal.status === 403, `HTTP ${traversal.status}`)

const pathish = await call('POST', '/so101/file/read', { id: 'C:\\Windows\\System32\\drivers\\etc\\hosts' })
check('a raw path is refused too', pathish.status === 403, `HTTP ${pathish.status}`)

const missing = await call('POST', '/so101/file/read', { id: 'agents-project' })
check('a listed file that is absent answers 404', missing.status === 404, `HTTP ${missing.status}`)

console.log('\n4. write is bounded by the same list')
const badWrite = await call('POST', '/so101/file/write', { id: 'not-on-the-list', content: 'x' })
check('writing an unlisted id is refused with 403', badWrite.status === 403, `HTTP ${badWrite.status}`)

const badType = await call('POST', '/so101/file/write', { id: 'skill', content: 42 })
check('a non-string body is refused with 400', badType.status === 400, `HTTP ${badType.status}`)

// Round-trip the skill file's own bytes: this exercises the write path without changing
// anything on disk, which a test has no business doing to an operator's configuration.
const roundTrip = await call('POST', '/so101/file/write', { id: 'skill', content: skill.json.content })
check('writing a listed file back answers 200', roundTrip.status === 200, `HTTP ${roundTrip.status}`)
const after = await call('POST', '/so101/file/read', { id: 'skill' })
check('and the content is unchanged', after.json && after.json.content === skill.json.content)

console.log('\n5. a malformed body is refused, not thrown')
const notJson = await call('POST', '/so101/file/read', 'this is not json', true)
check('a non-JSON body answered 400 rather than rejecting', notJson.status === 400, `HTTP ${notJson.status}`)

console.log('\n6. the bridge is unreachable, and that is a rendered state')
const status = await call('GET', '/so101/status')
check('GET /so101/status answers 200 (not a 5xx)', status.status === 200, `HTTP ${status.status}`)
check('and reports the bridge down', status.json && status.json.ok === false && status.json.bridge_reachable === false)
check('with a hint the operator can act on', !!(status.json && typeof status.json.hint === 'string' && status.json.hint.length > 40))

const frame = await call('GET', '/so101/frame?cam=overhead')
check('GET /so101/frame answers 503 with no bridge', frame.status === 503, `HTTP ${frame.status}`)

const move = await call('POST', '/so101/move_joints', { joints: { gripper: 200 } })
check('POST /so101/move_joints answers 503 with no bridge', move.status === 503, `HTTP ${move.status}`)

console.log('\n7. unknown routes are named, not swallowed')
const unknown = await call('GET', '/so101/nope')
check('an unknown route answers 404', unknown.status === 404, `HTTP ${unknown.status}`)
check('and echoes the route it did not find', !!(unknown.json && /nope/.test(unknown.json.error || '')))

const outside = await call('GET', '/somewhere-else')
check('a path outside the prefix is not claimed', outside.status === 404, `HTTP ${outside.status}`)

console.log()
if (failures.length > 0) {
  console.log(`FAILED: ${failures.length} check(s): ${failures.join(' | ')}`)
  process.exit(1)
}
console.log('ROUTES VERIFIED — the panel API answers on every path, and the file surface is an allow-list.')
