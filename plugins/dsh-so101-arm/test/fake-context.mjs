/**
 * A recording Cordis context that enforces the one runtime rule a hand-rolled stub silently
 * does not: `ctx.<service>` is readable only when the plugin declared that service in
 * `inject`.
 *
 * This file exists because of a shipped bug. `load-check.mjs` used to hand `apply()` a stub
 * with `tools` sitting on it as a plain property, so `inject = []` together with
 * `ctx.tools.register(...)` passed every check — and then took the entire profile tree down
 * at boot with:
 *
 *     cannot get property "tools" without inject
 *
 * A stub that is more permissive than the runtime is worse than no stub at all: it converts
 * a crash-on-startup into a green check. The lesson is not "be careful with inject" but
 * "make the fake context as strict as the real one", so this helper is the single place that
 * decides what the plugin is allowed to touch.
 */

/**
 * Build a guarded fake context.
 *
 * @param {object} options
 * @param {readonly string[]} options.inject  the plugin's declared inject list, read from the
 *   module under test so the stub cannot drift from what the plugin actually declares.
 * @param {Record<string, unknown>} options.services  service name -> implementation. Every
 *   key here is guarded: reaching it through `ctx.<name>` without a matching `inject` entry
 *   throws the same error the real Guard throws.
 * @param {{ provided?: string | null, effects?: unknown[] }} [options.sink]  records
 *   `ctx.provide` and `ctx.effect` so a test can assert on them.
 * @returns {object} the guarded context.
 */
export function guardContext({ inject, services, sink = {} }) {
  if (sink.provided === undefined) sink.provided = null
  if (sink.deferred === undefined) sink.deferred = []
  const declared = new Set(inject)

  // Cordis infrastructure that is NOT a service: always reachable, never guarded. Keeping it
  // separate from `services` is what makes the guard precise rather than merely strict.
  const infrastructure = {
    get(name) {
      return services[name]
    },
    // `ctx.inject(deps, cb)` is Cordis' `ctx.plugin({ inject: deps, apply: cb })`: the callback
    // runs on a CHILD fiber once every named service is present, and never runs at all if one
    // never appears. Both halves are load-bearing. The guarded child keeps the inject rule
    // enforced inside the callback, and an absent dependency must leave the callback unrun —
    // "the route was never registered, and nothing said so" is precisely the failure this stub
    // has to be able to reproduce instead of hiding.
    inject(deps, callback) {
      const missing = deps.filter((name) => !Object.prototype.hasOwnProperty.call(services, name))
      if (missing.length > 0) {
        sink.deferred.push({ deps: [...deps], missing })
        return { dispose() {} }
      }
      callback(guardContext({ inject: deps, services, sink }))
      return { dispose() {} }
    },
    effect(callback) {
      const disposer = callback()
      const owned = typeof disposer === 'function' ? disposer : () => {}
      if (Array.isArray(sink.effects)) sink.effects.push(owned)
      return owned
    },
    on() {
      return () => {}
    },
    provide(key) {
      sink.provided = key
      return () => {}
    },
    logger: {
      info() {},
      warn() {},
      error() {},
    },
  }

  return new Proxy(infrastructure, {
    get(target, property, receiver) {
      if (typeof property !== 'string') return Reflect.get(target, property, receiver)
      if (Object.prototype.hasOwnProperty.call(target, property)) return target[property]
      if (Object.prototype.hasOwnProperty.call(services, property)) {
        if (!declared.has(property)) {
          // Verbatim the runtime message, so a failure here reads the same as the crash it
          // is standing in for.
          throw new Error(`cannot get property "${property}" without inject`)
        }
        return services[property]
      }
      return undefined
    },
  })
}
