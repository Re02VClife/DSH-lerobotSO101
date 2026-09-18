"""Hold the bridge's torque lease open while an agent drives the arm.

The bridge releases servo torque when no client has checked in for `--watchdog-s`
(5 s by default).  That is the right default for a script, which loops tightly, and the
wrong default for an agent, which is silent for tens of seconds between tool calls.  So
the lease expires between every step, the servos go limp -- and these servos have no
brake, so the arm sags under gravity.  The next tool call then finds the arm somewhere
else entirely, re-commands from there, and the sequence never converges.  From outside it
looks like the arm "doesn't move".

Run this in the background while driving the arm:

    .\\.venv\\Scripts\\python.exe bridge\\keepalive.py --minutes 30

It only ever POSTs /heartbeat.  It never enables torque, so it cannot start a motion, and
it cannot undo an estop -- an estopped arm stays limp while this runs, because nothing
re-enables torque except an explicit move or /enable.

The proper home for this is inside the DSH plugin, so the harness holds its own lease
without a sidecar process.  Until that is loaded, this is the stand-in.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import urllib.error
import urllib.request


def beat(http: str, timeout: float) -> None:
    req = urllib.request.Request(f"{http}/heartbeat", data=b"{}",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=timeout) as fh:
        json.loads(fh.read().decode("utf-8"))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--http", default="http://127.0.0.1:8731")
    ap.add_argument("--interval", type=float, default=1.5,
                    help="seconds between beats; must stay well under --watchdog-s")
    ap.add_argument("--minutes", type=float, default=30.0)
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()

    deadline = time.time() + args.minutes * 60.0
    beats = 0
    misses = 0
    while time.time() < deadline:
        started = time.time()
        try:
            beat(args.http, args.timeout)
            beats += 1
            if misses:
                print(f"reconnected after {misses} missed beat(s)", flush=True)
                misses = 0
        except (urllib.error.URLError, OSError, ValueError) as exc:
            misses += 1
            # A restart or a long move can drop a beat or two; that is not fatal, because
            # the loop simply keeps trying.  Only a run of misses means the bridge is gone.
            if misses in (1, 5, 20):
                print(f"beat failed ({misses} in a row): {exc}", flush=True)
            if misses >= 20:
                print("bridge unreachable for 20 beats; giving up", file=sys.stderr,
                      flush=True)
                return 1
        # Keep the cadence from the start of the attempt, so a slow request does not push
        # the next beat out past the watchdog timeout.
        time.sleep(max(0.0, args.interval - (time.time() - started)))
    print(f"kept the lease alive for {args.minutes:g} min ({beats} beats)", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
