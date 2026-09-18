"""Watch every joint's tick range while a human sweeps the arm by hand.

Why measure instead of assume
-----------------------------
`arm_zero.py --mode soft` asks you to place the arm at the middle pose and press enter, which
means knowing where the middle is -- for six joints, by eye. Sweeping each joint through its
real travel answers the same question with the arm's own encoders, and it also records the
actual span, which is the only thing that says whether a joint is mechanically restricted or
the reading is plausible at all.

The middle is then `(min + max) / 2` in raw ticks, which is exactly what `middle_ticks` holds.

Torque must be OFF, or the arm cannot be moved by hand. The bridge in read-only mode leaves it
off; this refuses to start if any joint reports torque enabled, because a joint holding
position will not move and the recorded range would be a lie.

In practice the joints will be FLOPPY and gravity will pull the arm down: support it while you
move it, especially the shoulder and elbow.

Run:  .venv\\Scripts\\python.exe calib\\range_calib.py
Sweep every joint slowly to both hard limits, then press Ctrl+C.
"""
from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from bridge.servo_bus import JOINT_IDS, TICKS_PER_TURN  # noqa: E402

JOINTS = list(JOINT_IDS.keys())

#: A single-turn servo wraps at 4096 ticks, so a recorded span wider than half a turn almost
#: certainly means the travel crossed that boundary and min/max are the wrong pair.
WRAP_SUSPECT = TICKS_PER_TURN // 2


def fetch_state(base: str) -> dict:
    with urllib.request.urlopen(base + "/state", timeout=5) as response:
        return json.loads(response.read().decode("utf-8"))


def main() -> None:
    ap = argparse.ArgumentParser(description="Record each joint's tick range by hand-sweeping it.")
    ap.add_argument("--base", default="http://127.0.0.1:8731", help="bridge base URL")
    ap.add_argument("--hz", type=float, default=10.0, help="samples per second")
    ap.add_argument("--out", default="calib/out/joint_range.json")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="stop on its own after N seconds (0 = until Ctrl+C)")
    ap.add_argument("--plain", action="store_true",
                    help="scroll instead of redrawing; use if the terminal mangles ANSI")
    args = ap.parse_args()

    try:
        payload = fetch_state(args.base)
    except (urllib.error.URLError, TimeoutError) as exc:
        print(f"cannot reach the bridge at {args.base}: {exc}")
        print("start it read-only:  .venv\\Scripts\\python.exe bridge\\server.py")
        raise SystemExit(1)
    if not payload.get("ok"):
        print("bridge answered /state with an error:", payload)
        raise SystemExit(1)

    inner = payload["state"]
    if inner.get("torque_enabled"):
        print("REFUSING: a joint reports torque enabled.")
        print("A joint holding position cannot be moved by hand, so its recorded range would be")
        print("a lie. Start the bridge read-only (no --allow-motion) and try again.")
        raise SystemExit(2)

    lows: dict[str, int | None] = {name: None for name in JOINTS}
    highs: dict[str, int | None] = {name: None for name in JOINTS}
    samples = 0
    started = time.time()
    period = 1.0 / max(0.5, args.hz)
    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    last_write = 0.0
    drawn = 0

    def snapshot(final: bool) -> dict:
        return {
            "format": "ds101-so101-joint-range",
            "version": 1,
            "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "source": args.base,
            "ticks_per_turn": TICKS_PER_TURN,
            "samples": samples,
            "duration_s": round(time.time() - started, 2),
            "final": final,
            "range": {
                name: {
                    "min": lows[name],
                    "max": highs[name],
                    "span": None if lows[name] is None else highs[name] - lows[name],
                    "middle": None if lows[name] is None else (lows[name] + highs[name]) // 2,
                }
                for name in JOINTS
            },
        }

    print("DS101 SO-101 joint range recorder")
    print("Sweep EVERY joint slowly to both of its hard limits. Ctrl+C when done.")
    print("The arm is floppy -- support it so it does not fall.\n")
    header = f"{'joint':<16}{'now':>7}{'min':>7}{'max':>7}{'span':>7}{'deg':>7}"
    print(header)
    print("-" * len(header))
    drawn = 1

    try:
        while True:
            if args.seconds and time.time() - started >= args.seconds:
                break
            try:
                ticks = fetch_state(args.base)["state"].get("raw_ticks") or {}
            except (urllib.error.URLError, TimeoutError, KeyError):
                time.sleep(period)
                continue

            for name in JOINTS:
                tick = ticks.get(name)
                if tick is None:
                    continue
                if lows[name] is None or tick < lows[name]:
                    lows[name] = tick
                if highs[name] is None or tick > highs[name]:
                    highs[name] = tick
            samples += 1

            body = []
            for name in JOINTS:
                lo, hi = lows[name], highs[name]
                span = 0 if lo is None else hi - lo
                body.append(f"{name:<16}{ticks.get(name, 0):>7}{lo if lo is not None else 0:>7}"
                            f"{hi if hi is not None else 0:>7}{span:>7}"
                            f"{span * 360.0 / TICKS_PER_TURN:>7.1f}")
            body.append("")
            body.append(f"samples {samples}   {time.time() - started:6.1f}s"
                        + (f"   / {args.seconds:.0f}s" if args.seconds else ""))

            if args.plain:
                print("\n".join(body))
            else:
                # Redraw in place. Modern Windows terminals all speak ANSI; --plain is the
                # escape hatch for one that does not.
                sys.stdout.write(f"\x1b[{drawn}A" if drawn else "")
                sys.stdout.write("\n".join(body) + "\n")
                drawn = len(body)
            sys.stdout.flush()

            now = time.time()
            if now - last_write > 1.0:
                # Written every second, not every sample: this file is for someone else to read
                # while the sweep is still going, and 10 writes a second buys nothing.
                out_path.write_text(json.dumps(snapshot(False), indent=2), encoding="utf-8")
                last_write = now
            time.sleep(period)
    except KeyboardInterrupt:
        pass

    result = snapshot(True)
    out_path.write_text(json.dumps(result, indent=2), encoding="utf-8")

    print("\n\n=== recorded range ===")
    print(f"{'joint':<16}{'min':>7}{'max':>7}{'span':>7}{'middle':>8}{'deg span':>10}")
    for name in JOINTS:
        r = result["range"][name]
        deg = "" if r["span"] is None else f"{r['span'] * 360.0 / TICKS_PER_TURN:.1f}"
        print(f"{name:<16}{str(r['min']):>7}{str(r['max']):>7}{str(r['span']):>7}"
              f"{str(r['middle']):>8}{deg:>10}")
    print(f"\nwritten: {out_path}   ({samples} samples over {result['duration_s']:.1f}s)")

    if any((r["span"] or 0) >= WRAP_SUSPECT for r in result["range"].values()):
        print("\nWARNING: a span reached half a turn or more. A single-turn servo wraps at 4096,")
        print("so that joint probably crossed the boundary and its min/max are not a real pair.")
    idle = [n for n in JOINTS if (result["range"][n]["span"] or 0) < 30]
    if idle:
        print(f"\nWARNING: barely moved (span < 30 ticks): {', '.join(idle)}")
        print("Their middle would just be wherever the arm happened to sit.")

    # Only hold the window open for a human. With stdin not a terminal (a test run, a pipe)
    # waiting for a keypress would hang whatever called this.
    if sys.stdin.isatty():
        # Guarded: isatty() can report true while stdin is still not readable (a redirected
        # pipe under an interactive shell), and an EOFError here would abort an otherwise
        # finished run -- after the results are already on disk, which reads much worse than
        # it is.
        try:
            input("\nPress ENTER to close this window... ")
        except EOFError:
            pass


if __name__ == "__main__":
    main()
