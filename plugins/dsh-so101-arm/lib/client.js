/**
 * Browser half of dsh-so101-arm: the operator's console for the DS101 bench.
 *
 * ## Why this file has an unusual shape
 *
 * This is NOT a normally-imported ES module. The DSH client module system loads every
 * `dsh.client` bundle as a script whose only job is to call
 * `window.__ModuleLoader__.load({ id, factory })`; the loader then calls `factory(require)`
 * on first import and caches the returned `exports`. React comes from the platform's module
 * table (`require("react")`), not from node_modules, which is why there is no bundler step
 * for this file and no `import` statements anywhere in it.
 *
 * Consequences worth knowing before editing:
 *
 *   * `require` resolves ONLY platform seed words and other registered bundle factories.
 *     A stray `require("node:fs")` here fails at load with "missed the module table".
 *   * JSX is not available — this file is never transformed — so every element is
 *     `React.createElement`.
 *   * The export contract is the same object a Cordis plugin module would export:
 *     `{ name?, inject, apply }`.
 *
 * ## Where the panels live
 *
 * Tabs are contributed to the DSH right-hand column through the `ctx.betterSidebar`
 * registry that `dsh-better-sidebar` publishes. That service exists on the CLIENT half
 * only; the host half has no such service and reaches the bench over its own HTTP routes
 * instead. Registration must be wrapped in `ctx.effect(...)` so the disposer runs on
 * unload — without it, a plugin reload leaves the old registration behind and the next
 * activation throws "already registered".
 *
 * ## How data actually moves
 *
 * Every byte comes from the host half's same-origin routes under `/so101/`. That is
 * deliberate: same-origin means no CORS negotiation, no port discovery in the browser, and
 * no credentials crossing a loopback boundary. The host half owns the conversation with
 * the Python bridge; this half only renders JSON and `<img>` tags.
 *
 * A camera frame is an `<img src="/so101/frame?cam=overhead&t=...">` — the host half
 * fetches a frame from the bridge, reads the PNG off disk, and streams it back. The `t`
 * query is what defeats the browser cache.
 */
window.__ModuleLoader__.load({
  id: 'dsh-so101-arm',
  factory: (require) => {
    const module = { exports: {} }
    const exports = module.exports

    const React = require('react')
    const h = React.createElement

    /** The host half's route prefix. Must match `ROUTE_PREFIX` in lib/index.js. */
    const API = '/so101'

    /** The six servo names, in the order the bench labels them. */
    const JOINTS = [
      ['shoulder_pan', '底座旋转'],
      ['shoulder_lift', '大臂抬升'],
      ['elbow_flex', '小臂'],
      ['wrist_flex', '腕俯仰'],
      ['wrist_roll', '腕旋转'],
      ['gripper', '夹爪'],
    ]

    /**
     * One JSON call against the host half.
     *
     * Never throws: a bridge that is down, a route that 403s and a refusal from the servo
     * envelope are all things the panel must render, not exceptions to propagate into
     * React. The caller gets a discriminated result and decides what to show.
     */
    async function api(path, body) {
      try {
        const res = await fetch(API + path, {
          method: body === undefined ? 'GET' : 'POST',
          headers: body === undefined ? undefined : { 'content-type': 'application/json' },
          body: body === undefined ? undefined : JSON.stringify(body),
        })
        const text = await res.text()
        let json
        try {
          json = JSON.parse(text)
        } catch {
          return { ok: false, error: `non-JSON reply (HTTP ${res.status})`, detail: text.slice(0, 300) }
        }
        return json
      } catch (error) {
        return { ok: false, error: String((error && error.message) || error) }
      }
    }

    /**
     * Poll `fn` every `ms` while `active`, and once immediately on activation.
     *
     * The `active` gate is the important part: a collapsed panel or a background tab must
     * not keep opening the RealSense, because only one process may hold that camera and each
     * frame costs real milliseconds.
     */
    function usePoll(fn, ms, active) {
      const ref = React.useRef(fn)
      ref.current = fn
      React.useEffect(() => {
        if (!active) return undefined
        let alive = true
        const tick = async () => {
          if (!alive) return
          await ref.current()
        }
        tick()
        const id = setInterval(tick, ms)
        return () => {
          alive = false
          clearInterval(id)
        }
      }, [ms, active])
    }

    // ── shared bits ────────────────────────────────────────────────────────────────

    const S = {
      root: {
        height: '100%',
        minHeight: 0,
        overflow: 'auto',
        padding: '10px 12px 24px',
        fontSize: 12,
        lineHeight: 1.5,
        color: 'var(--dsw-alias-text-1, inherit)',
        background: 'var(--dsw-alias-bg-layer-1, transparent)',
        boxSizing: 'border-box',
      },
      card: {
        border: '1px solid var(--dsw-alias-border-1, rgba(128,128,128,.3))',
        borderRadius: 8,
        padding: '8px 10px',
        marginBottom: 10,
        background: 'var(--dsw-alias-bg-layer-2, rgba(128,128,128,.06))',
      },
      h: { fontSize: 12, fontWeight: 600, margin: '0 0 6px' },
      row: { display: 'flex', alignItems: 'center', gap: 8, marginBottom: 6 },
      btn: {
        font: 'inherit',
        padding: '3px 9px',
        borderRadius: 6,
        cursor: 'pointer',
        border: '1px solid var(--dsw-alias-border-1, rgba(128,128,128,.4))',
        background: 'var(--dsw-alias-bg-layer-1, transparent)',
        color: 'inherit',
      },
      dim: { opacity: 0.65 },
      img: {
        width: '100%',
        display: 'block',
        borderRadius: 6,
        border: '1px solid var(--dsw-alias-border-1, rgba(128,128,128,.3))',
        background: '#111',
      },
      pre: {
        margin: 0,
        fontSize: 11,
        whiteSpace: 'pre-wrap',
        wordBreak: 'break-all',
        fontFamily: 'ui-monospace, Consolas, monospace',
      },
      input: {
        font: 'inherit',
        width: '100%',
        boxSizing: 'border-box',
        color: 'inherit',
        background: 'var(--dsw-alias-bg-layer-1, transparent)',
        border: '1px solid var(--dsw-alias-border-1, rgba(128,128,128,.4))',
        borderRadius: 6,
        padding: '4px 6px',
      },
    }

    function Btn(props) {
      const { children, onClick, disabled, title } = props
      return h(
        'button',
        {
          style: { ...S.btn, ...(disabled ? S.dim : null), ...(props.style || null) },
          onClick,
          disabled,
          title,
          type: 'button',
        },
        children,
      )
    }

    function Pill(props) {
      const colour = props.ok ? '#2ea043' : '#d29922'
      return h(
        'span',
        {
          style: {
            display: 'inline-block',
            width: 7,
            height: 7,
            borderRadius: 7,
            background: colour,
            marginRight: 5,
          },
        },
        null,
      )
    }

    /** Accept either a scalar (what a future bridge might send) or a per-joint map. */
    function numbersOf(values) {
      if (typeof values === 'number') return [values]
      if (values === null || typeof values !== 'object') return []
      return Object.keys(values)
        .map((k) => values[k])
        .filter((v) => typeof v === 'number')
    }

    /**
     * Render a per-joint map as `4.5–4.6 V`, collapsing to `4.6 V` when every joint agrees.
     *
     * The bridge reports `voltage` and `temperature` as `{ joint: value }` maps, NOT scalars —
     * see `state()` in bridge/driver.py. The first version of this panel assumed scalars and
     * rendered "[object Object] V". It shipped because the render test only ever exercised the
     * component with `status === null`, so the data branch had never run against a real
     * payload: rendering once is not the same as rendering with something to render.
     */
    function span(values, unit, fmt) {
      const format = fmt || ((v) => String(Math.round(v * 10) / 10))
      const nums = numbersOf(values)
      if (nums.length === 0) return null
      const lo = Math.min(...nums)
      const hi = Math.max(...nums)
      return lo === hi ? `${format(lo)} ${unit}` : `${format(lo)}–${format(hi)} ${unit}`
    }

    /** Highest value of a per-joint map, e.g. `最高 30 °C`. */
    function maxText(values, unit) {
      const nums = numbersOf(values)
      if (nums.length === 0) return null
      return `最高 ${Math.max(...nums)} ${unit}`
    }

    // ── tab 1: the bench ───────────────────────────────────────────────────────────

    function BenchView(props) {
      const visible = props.visible
      const [status, setStatus] = React.useState(null)
      const [busy, setBusy] = React.useState(null)
      const [note, setNote] = React.useState(null)
      const [targets, setTargets] = React.useState(null)
      const [frameTick, setFrameTick] = React.useState(() => Date.now())
      const [frameSrc, setFrameSrc] = React.useState(null)
      const [frameError, setFrameError] = React.useState(null)
      /** True while a frame request is in flight, so the poll cannot queue stale grabs. */
      const inFlight = React.useRef(false)
      const [live, setLive] = React.useState(false)
      const [cam, setCam] = React.useState('overhead')
      const [log, setLog] = React.useState([])
      /** Measured joint travel from the calibration file: { joint: [minDeg, maxDeg] }. */
      const [limits, setLimits] = React.useState(null)
      /** The operator's recorded HOME pose in degrees; the sliders open on it. */
      const [home, setHome] = React.useState(null)

      const say = React.useCallback((text, bad) => {
        setLog((prev) => [{ t: new Date().toLocaleTimeString(), text, bad: !!bad }, ...prev].slice(0, 40))
      }, [])

      const refresh = React.useCallback(async () => {
        const res = await api('/status')
        setStatus(res)
        if (res && res.ok && res.state && res.state.joint_deg) {
          // Only seed the sliders when the operator is not mid-edit, otherwise a poll
          // would yank a slider back under their finger.
          setTargets((prev) => prev ?? { ...res.state.joint_deg })
        }
        return res
      }, [])

      // Status is cheap (no camera); the frame is not, so they poll on different clocks
      // and the camera only runs while the operator asked for a live view.
      // One probe on mount, deliberately INDEPENDENT of `visible`.
      //
      // If the tab's visibility contract ever fails to arrive, the panel must still be able to
      // say why it is offline instead of sitting on its initial state forever -- which is
      // exactly how a healthy bridge and a registered route both got misread as "bridge 未运行".
      // The periodic poll below stays gated on `visible`: a hidden tab must not keep opening
      // the RealSense.
      React.useEffect(() => {
        refresh()
      }, [refresh])

      // Slider bounds come from the MEASURED joint travel in the calibration file, never from a
      // symmetric guess. The panel used to offer +/-180 for every joint while shoulder_pan's real
      // travel is 167.8 deg: most of that slider was a range the arm cannot enter, and dragging
      // into it only stalls the servo against a hard limit.
      React.useEffect(() => {
        let cancelled = false
        api('/file/read', { id: 'calibration' })
          .then((res) => {
            if (cancelled || !res || !res.ok || typeof res.content !== 'string') return
            let calib
            try {
              calib = JSON.parse(res.content)
            } catch {
              return
            }
            if (calib && calib.joint_limits_deg) setLimits(calib.joint_limits_deg)
            if (calib && calib.home_deg) setHome(calib.home_deg)
          })
          .catch(() => {
            /* an unreadable calibration leaves the fallback range, which is at least visible */
          })
        return () => {
          cancelled = true
        }
      }, [])

      // Open on HOME, once. The live-pose seed in refresh() lands first (it is cheaper), so this
      // replaces it exactly once when the calibration arrives -- and then never again, or the
      // 回读 button and every manual drag would be silently undone on the next poll.
      const homeSeeded = React.useRef(false)
      React.useEffect(() => {
        if (homeSeeded.current || home === null || targets === null) return
        homeSeeded.current = true
        setTargets({ ...home })
      }, [home, targets])

      usePoll(refresh, 2000, visible)
      // 1000 ms. 300 ms was too aggressive: every frame is a whole /look round trip -- open the
      // RealSense, align, encode a PNG, stream it back -- so 3.3 Hz spent a large slice of a core
      // on the live view alone. 1 Hz is still 2.5x the old 2500 ms and is what was asked for.
      usePoll(async () => setFrameTick(Date.now()), 1000, visible && live)

      // Frames are FETCHED, not pointed at by an <img src>.
      //
      // An <img> that fails hands the panel no status code at all: a route the host never
      // exposed, a 403 from the web carrier, and a bridge that is genuinely down are
      // indistinguishable -- all three render as "抓帧失败：bridge 未运行", which blames the one
      // process that was actually healthy. fetch() can say which one it was.
      React.useEffect(() => {
        // Gated on `visible`, for the same reason the live poll is: the RealSense is the one
        // resource only a single process may hold, and a hidden tab must not open it. The
        // /status probe above is the deliberate exception -- cheap, and it touches no camera.
        if (!visible) return undefined
        // Never let frames pile up. At a 300 ms poll a slow grab can still be in flight when the
        // next tick fires, and a queue of stale requests would show an image far older than
        // "live" implies.
        if (inFlight.current) return undefined
        inFlight.current = true
        let cancelled = false
        let objectUrl = null
        fetch(`${API}/frame?cam=${cam}&t=${frameTick}`)
          .then(async (res) => {
            if (!res.ok) {
              const body = await res.text().catch(() => '')
              throw new Error(`HTTP ${res.status}${body ? `: ${body.slice(0, 200)}` : ''}`)
            }
            const blob = await res.blob()
            if (cancelled) return
            objectUrl = URL.createObjectURL(blob)
            setFrameSrc(objectUrl)
            setFrameError(null)
          })
          .catch((error) => {
            if (cancelled) return
            setFrameSrc(null)
            setFrameError(String((error && error.message) || error))
          })
          .finally(() => {
            inFlight.current = false
          })
        return () => {
          cancelled = true
          if (objectUrl !== null) URL.revokeObjectURL(objectUrl)
        }
      }, [cam, frameTick, visible])

      const state = status && status.ok ? status.state : null
      const health = status && status.health ? status.health : null
      const online = !!(status && status.ok)
      const motion = !!(health && health.motion_allowed)

      // Re-grab a frame when the bridge comes BACK, not only when the tab opens.
      //
      // The frame fetch is deliberately not on the poll -- the RealSense must not be opened
      // every 2 seconds -- so without this a bridge restart leaves "bridge 在线" sitting above
      // a stale failure image until the operator notices and clicks 抓帧. One more way for a
      // working bench to look broken. Only the false -> true edge fires, so a healthy bridge
      // does not re-open the camera on every status poll.
      const wasOnline = React.useRef(false)
      React.useEffect(() => {
        if (online && !wasOnline.current) setFrameTick(Date.now())
        wasOnline.current = online
      }, [online])

      const act = React.useCallback(
        async (path, body, label) => {
          setBusy(label)
          setNote(null)
          const res = await api(path, body === undefined ? {} : body)
          setBusy(null)
          const ok = res && res.ok
          const detail = res && (res.error || res.status || (res.message || ''))
          setNote({ ok, text: ok ? `${label} 成功` : `${label} 被拒绝: ${detail || '未知原因'}` })
          say(`${label}: ${ok ? 'ok' : (res && res.error) || 'refused'}`, !ok)
          await refresh()
          return res
        },
        [refresh, say],
      )

      const dirty = !!(targets && state &&
        JOINTS.some(([name]) => {
          const cur = state.joint_deg ? state.joint_deg[name] : undefined
          const want = targets[name]
          return typeof cur === 'number' && typeof want === 'number' && Math.abs(cur - want) > 0.2
        }))

      return h(
        'div',
        { style: S.root },
        // status header
        h(
          'div',
          { style: S.card },
          h(
            'div',
            { style: { ...S.row, marginBottom: 4 } },
            h(Pill, { ok: online }),
            // The reason rides ON the headline, not only in a line further down. "bridge 离线"
            // while the bridge was healthy is exactly how this panel wasted a whole session:
            // the one fact that would have explained it was rendered somewhere easy to miss.
            h(
              'strong',
              null,
              online
                ? 'bridge 在线'
                : `bridge 离线 — ${(status && (status.error || status.message)) || '请求尚未返回'}`,
            ),
            h('span', { style: { ...S.dim, marginLeft: 'auto' } }, health ? `端口 ${health.port}` : ''),
          ),
          !online &&
            h(
              'div',
              { style: S.dim },
              'bridge 未运行。它独占串口和 RealSense，只能有一个实例：',
              h('div', { style: { ...S.pre, marginTop: 4 } }, 'python bridge\\server.py --allow-motion'),
              // The transport failure is the whole diagnosis, and it is NOT the same fact as
              // "the bridge is down": our own route missing (404), the host gateway refusing
              // (403), a CSP-blocked request and a genuinely absent bridge all used to
              // collapse into that one sentence, which points at the wrong process.
              status &&
                h(
                  'div',
                  { style: { ...S.dim, marginTop: 4, color: '#d29922' } },
                  `/so101/status → ${status.error || status.message || 'no error field'}` +
                    `${status.detail ? `: ${status.detail}` : ''}`,
                ),
            ),
          online &&
            h(
              'div',
              null,
              h('div', null, motion ? '运动已允许 (--allow-motion)' : '只读模式：会拒绝所有动作'),
              state &&
                h(
                  'div',
                  { style: S.dim },
                  [
                    span(state.voltage, 'V'),
                    maxText(state.temperature, '°C'),
                    Array.isArray(state.tool_mm)
                      ? `工具点 [${state.tool_mm.map((v) => Math.round(v)).join(', ')}] mm`
                      : null,
                  ]
                    .filter(Boolean)
                    .join(' · '),
                ),
            ),
        ),

        // cameras
        h(
          'div',
          { style: S.card },
          h(
            'div',
            { style: S.row },
            h('strong', null, '相机'),
            h(
              'select',
              {
                style: { ...S.input, width: 'auto' },
                value: cam,
                onChange: (e) => setCam(e.target.value),
              },
              h('option', { value: 'overhead' }, '顶部 D435'),
              h('option', { value: 'wrist' }, '腕部'),
            ),
            h(Btn, { onClick: () => setLive((v) => !v), title: '每 2.5 秒重新抓帧' }, live ? '停止' : '实时'),
            h(Btn, { onClick: () => setFrameTick(Date.now()) }, '抓帧'),
          ),
          h('img', {
            style: S.img,
            alt: '相机画面（需要 bridge 在线）',
            src: frameSrc || undefined,
          }),
          frameError !== null &&
            h(
              'div',
              { style: { ...S.dim, marginTop: 4, color: '#d29922' } },
              `抓帧失败：${frameError}`,
            ),
          h('div', { style: { ...S.dim, marginTop: 4 } }, cam === 'wrist'
            ? '腕部相机只有接了真实硬件才有画面。'
            : '顶部相机：固定安装的 D435 俯视整个工作区。'),
        ),

        // joints
        h(
          'div',
          { style: S.card },
          h(
            'div',
            { style: S.row },
            h('strong', null, '关节角度'),
            // Say where the bounds and the starting pose came from. BOTH have a silent fallback,
            // and a silent fallback is exactly how "the sliders open on home" became "the sliders
            // show wherever the arm happens to be" without a word appearing anywhere.
            h('span', { style: { ...S.dim, marginLeft: 'auto' } },
              `限位${limits ? '✓' : '×'} home${home ? '✓' : '×'} · `),
            // Three states, not two: with `state` still null (first poll in flight, or the bridge
            // down) the old two-way test reported "与机械臂一致" about a pose it had never read.
            h('span', { style: S.dim }, !state ? '未读到' : dirty ? '未应用' : '与机械臂一致'),
            h(Btn, { onClick: () => setTargets(state ? { ...state.joint_deg } : null) }, '回读'),
          ),
          JOINTS.map(([name, label]) => {
            const cur = state && state.joint_deg ? state.joint_deg[name] : undefined
            const want = targets && typeof targets[name] === 'number' ? targets[name] : (typeof cur === 'number' ? cur : 0)
            // Measured travel when the calibration is readable, a visible fallback otherwise.
            const lim = limits && Array.isArray(limits[name]) ? limits[name] : null
            const lo = lim ? Math.floor(lim[0]) : name === 'gripper' ? 0 : -180
            const hi = lim ? Math.ceil(lim[1]) : name === 'gripper' ? 360 : 180
            return h(
              'div',
              { key: name, style: { marginBottom: 4 } },
              h(
                'div',
                { style: { display: 'flex', gap: 6 } },
                h('span', { style: { minWidth: 66 } }, label),
                h('span', { style: { ...S.dim, minWidth: 42, textAlign: 'right' } }, `${want.toFixed(1)}°`),
                typeof cur === 'number' &&
                  h('span', { style: { ...S.dim, marginLeft: 'auto' } }, `当前 ${cur.toFixed(1)}°`),
              ),
              h('input', {
                type: 'range',
                min: lo,
                max: hi,
                step: 1,
                value: want,
                style: { width: '100%' },
                onChange: (e) =>
                  setTargets((prev) => ({ ...(prev || {}), [name]: Number(e.target.value) })),
              }),
            )
          }),
          h(
            'div',
            { style: { ...S.row, marginTop: 8, marginBottom: 0 } },
            h(
              Btn,
              {
                onClick: () => act('/move_joints', { joints: targets }, '下发关节角'),
                disabled: !online || !targets || busy !== null,
                title: motion ? '' : 'bridge 处于只读模式',
              },
              busy === '下发关节角' ? '下发中…' : '应用到机械臂',
            ),
            h(Btn, { onClick: () => act('/home', {}, '回 HOME'), disabled: !online || busy !== null }, '回 HOME'),
            h(
              Btn,
              {
                onClick: () => act('/gripper', { openness_pct: 100 }, '张开夹爪'),
                disabled: !online || busy !== null,
              },
              '张开',
            ),
            h(
              Btn,
              {
                onClick: () => act('/gripper', { openness_pct: 0 }, '闭合夹爪'),
                disabled: !online || busy !== null,
              },
              '闭合',
            ),
          ),
          note &&
            h(
              'div',
              { style: { marginTop: 6, color: note.ok ? '#2ea043' : '#d29922' } },
              note.text,
            ),
          h(
            'div',
            { style: { ...S.dim, marginTop: 6 } },
            '这是软件层面的控制台，不是急停。真实硬件运动前必须先有物理急停。',
          ),
        ),

        // log
        h(
          'div',
          { style: S.card },
          h('div', { style: S.h }, '操作记录'),
          log.length === 0 && h('div', { style: S.dim }, '（还没有操作）'),
          log.map((entry, i) =>
            h(
              'div',
              { key: `${entry.t}-${i}`, style: { color: entry.bad ? '#d29922' : 'inherit' } },
              h('span', { style: S.dim }, `${entry.t}  `),
              entry.text,
            ),
          ),
        ),
      )
    }

    // ── tab 2: configuration files ─────────────────────────────────────────────────

    function ConfigView(props) {
      const visible = props.visible
      const [files, setFiles] = React.useState(null)
      const [open, setOpen] = React.useState(null)
      const [draft, setDraft] = React.useState('')
      const [saving, setSaving] = React.useState(false)
      const [msg, setMsg] = React.useState(null)

      const load = React.useCallback(async () => {
        const res = await api('/files')
        if (res && res.ok) setFiles(res.files)
        else setMsg({ bad: true, text: (res && res.error) || '无法列出配置文件' })
      }, [])

      usePoll(load, 15000, visible)

      const openFile = React.useCallback(async (id) => {
        setMsg(null)
        const res = await api('/file/read', { id })
        if (res && res.ok) {
          setOpen(res)
          setDraft(res.content)
        } else {
          setMsg({ bad: true, text: (res && res.error) || '读取失败' })
        }
      }, [])

      const save = React.useCallback(async () => {
        if (!open) return
        setSaving(true)
        const res = await api('/file/write', { id: open.id, content: draft })
        setSaving(false)
        setMsg(res && res.ok ? { text: `已保存 ${open.path}` } : { bad: true, text: (res && res.error) || '保存失败' })
        if (res && res.ok) load()
      }, [open, draft, load])

      return h(
        'div',
        { style: S.root },
        h(
          'div',
          { style: S.card },
          h('div', { style: S.h }, '可编辑的配置文件'),
          h(
            'div',
            { style: S.dim },
            '只有这个清单里的文件能读写：面板暴露给页面的写权限必须是有边界的，' +
              '否则任何同源脚本都能改主机上的任意文件。',
          ),
        ),
        h(
          'div',
          { style: S.card },
          !files && h('div', { style: S.dim }, '加载中…'),
          files &&
            files.map((f) =>
              h(
                'div',
                { key: f.id, style: { ...S.row, marginBottom: 4 } },
                h(
                  'div',
                  { style: { minWidth: 0, flex: 1 } },
                  h('div', null, f.label),
                  h('div', { style: { ...S.dim, ...S.pre, fontSize: 10 } }, f.path),
                ),
                h('span', { style: { ...S.dim, whiteSpace: 'nowrap' } }, f.exists ? `${f.size} B` : '不存在'),
                h(Btn, { onClick: () => openFile(f.id), disabled: !f.exists }, '编辑'),
              ),
            ),
        ),
        msg && h('div', { style: { color: msg.bad ? '#d29922' : '#2ea043', marginBottom: 8 } }, msg.text),
        open &&
          h(
            'div',
            { style: S.card },
            h('div', { style: S.row }, h('strong', null, open.label)),
            h('textarea', {
              value: draft,
              spellCheck: false,
              onChange: (e) => setDraft(e.target.value),
              style: { ...S.input, height: 320, fontFamily: 'ui-monospace, Consolas, monospace' },
            }),
            h(
              'div',
              { style: { ...S.row, marginTop: 6, marginBottom: 0 } },
              h(Btn, { onClick: save, disabled: saving }, saving ? '保存中…' : '保存'),
              h(Btn, { onClick: () => setDraft(open.content) }, '还原'),
              h(Btn, { onClick: () => { setOpen(null); setMsg(null) } }, '关闭'),
            ),
          ),
      )
    }

    // ── registration ───────────────────────────────────────────────────────────────

    function apply(ctx) {
      const sidebar = ctx.get('betterSidebar')
      if (sidebar === undefined) {
        // dsh-better-sidebar is an optional peer: without it there is no right column to
        // contribute to, and the arm_* tools are unaffected. Say so once rather than
        // throwing, so a profile without the sidebar still boots.
        console.warn('[dsh-so101-arm] dsh-better-sidebar is not mounted; the bench panel is unavailable')
        return
      }

      ctx.effect(
        () =>
          sidebar.registerTab({
            id: 'dsh-so101-arm:bench',
            title: () => 'SO-101 机械臂',
            order: 20,
            single: true,
            description: () => '双相机画面、关节角度与夹爪控制',
            component: (props) => h(BenchView, { visible: props.visible }),
          }),
        'so101.benchTab',
      )

      ctx.effect(
        () =>
          sidebar.registerTab({
            id: 'dsh-so101-arm:config',
            title: () => 'SO-101 配置',
            order: 21,
            single: true,
            description: () => '查看与修改 skill、标定结果和 AGENTS.md',
            component: (props) => h(ConfigView, { visible: props.visible }),
          }),
        'so101.configTab',
      )
    }

    exports.apply = apply
    exports.inject = ['betterSidebar']
    /**
     * Test seam, mirroring `__rawToolDefinitions` on the host half.
     *
     * `client-check` feeds these the payload shapes the bridge actually produces. A rendering
     * test cannot catch a wrong shape guess, because a component rendered once with no data
     * never reaches its data branch — which is how "[object Object] V" shipped.
     */
    exports.__format = { span, maxText, numbersOf }
    return module.exports
  },
})
