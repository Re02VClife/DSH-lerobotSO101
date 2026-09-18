"""Probe whether an OpenAI-compatible endpoint actually accepts images.

Why this exists, and why it must be run BEFORE configuring a vision route:

DSH decides whether to attach an image from the model's declared input modalities. A
route using the pi-ai adapter can declare `defaultInput: [text, image]`. Nothing verifies
that declaration against the endpoint, and the failure is asymmetric:

  * under-claiming (declaring text only) costs a refusal that names the model -- annoying
    but harmless;
  * over-claiming admits the image, the harness commits the user message with that image
    durably into the session log, and only THEN does the provider reject it. The session
    now holds an image its model refuses, so it re-sends it every turn and model selection
    refuses to switch to a text-only model. Recovery needs another image-capable model, a
    fork before the image, or a new session.

So this probe goes out of band, on a throwaway request, where a failure costs nothing.

Usage
-----
    # key supplied inline (it is never written to disk by this script)
    python calib/vision_probe.py --base-url https://gateway.example/v1 --model deepseek-v4-flash --api-key sk-...

    # or point at a file holding just the key, so it stays out of shell history/process list
    python calib/vision_probe.py --base-url ... --model ... --api-key-file path/to/key.txt

The script sends two requests: a text-only control (does the endpoint even serve this
model from here?) and a 2x2 PNG whose colours are known, so a positive answer is
verifiable rather than a polite hallucination.
"""
from __future__ import annotations

import argparse
import base64
import json
import sys
import urllib.error
import urllib.request
import zlib
from pathlib import Path


def make_png(width: int, height: int, pixels: list[list[tuple[int, int, int]]]) -> bytes:
    """Minimal PNG encoder: no dependency, and the exact pixels are known."""
    raw = b""
    for row in pixels:
        raw += b"\x00" + b"".join(bytes(px) for px in row)
    compressed = zlib.compress(raw, 9)

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (len(data).to_bytes(4, "big") + tag + data
                + (zlib.crc32(tag + data) & 0xFFFFFFFF).to_bytes(4, "big"))

    ihdr = (width.to_bytes(4, "big") + height.to_bytes(4, "big")
            + bytes([8, 2, 0, 0, 0]))          # 8-bit, truecolour RGB
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", compressed) + chunk(b"IEND", b""))


def post(url: str, key: str, payload: dict, timeout: float) -> tuple[int, dict | str]:
    req = urllib.request.Request(
        url, data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json", "Authorization": f"Bearer {key}"},
        method="POST")
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return r.status, json.loads(r.read().decode())
    except urllib.error.HTTPError as e:
        return e.code, e.read().decode(errors="replace")[:800]
    except Exception as e:                       # noqa: BLE001
        return 0, f"{type(e).__name__}: {e}"


def main() -> None:
    ap = argparse.ArgumentParser(description="Probe an endpoint for real image support")
    ap.add_argument("--base-url", required=True, help="e.g. https://gateway.example/v1")
    ap.add_argument("--model", required=True)
    ap.add_argument("--api-key", default=None)
    ap.add_argument("--api-key-file", default=None)
    ap.add_argument("--timeout", type=float, default=60.0)
    args = ap.parse_args()

    key = args.api_key
    if key is None and args.api_key_file:
        key = Path(args.api_key_file).read_text(encoding="utf-8").strip()
    if not key:
        print("no credential: pass --api-key or --api-key-file. This probe never stores it.")
        raise SystemExit(2)

    url = args.base_url.rstrip("/") + "/chat/completions"
    print(f"endpoint: {url}\nmodel:    {args.model}\n")

    # ---- control: can this endpoint serve this model at all, in text mode? -----
    status, body = post(url, key, {
        "model": args.model,
        "max_tokens": 24,
        "messages": [{"role": "user", "content": "Reply with exactly: TEXT_OK"}],
    }, args.timeout)
    text_ok = status == 200
    print(f"[1] text control           -> HTTP {status}  {'OK' if text_ok else body}")
    if not text_ok:
        print("\nThe endpoint does not serve this model in text mode from here. Fix that")
        print("(base URL, model id, credential) before worrying about images.")
        raise SystemExit(1)

    # ---- the real question: a 2x2 image with known colours --------------------
    # top row red, red / bottom row blue, blue  -> a sighted model says red and blue
    png = make_png(2, 2, [[(255, 0, 0), (255, 0, 0)], [(0, 0, 255), (0, 0, 255)]])
    b64 = base64.b64encode(png).decode()
    status, body = post(url, key, {
        "model": args.model,
        "max_tokens": 64,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "text", "text": "A 2x2 pixel image. Name the two distinct colours "
                                         "you see, top row then bottom row. Be brief."},
                {"type": "image_url",
                 "image_url": {"url": f"data:image/png;base64,{b64}"}},
            ],
        }],
    }, args.timeout)

    print(f"[2] image request          -> HTTP {status}")
    if status != 200:
        print(f"    {body}")
        print("\nVERDICT: this endpoint does NOT accept images for this model.")
        print("  Do not declare [text, image]; leave the route as text-only.")
        raise SystemExit(1)

    try:
        text = body["choices"][0]["message"]["content"]
    except Exception:                            # noqa: BLE001
        text = json.dumps(body)[:400]
    print(f"    model said: {text!r}")
    lowered = str(text).lower()
    saw_red = "red" in lowered
    saw_blue = "blue" in lowered
    if saw_red and saw_blue:
        print("\nVERDICT: the endpoint really does read images (it named red and blue).")
        print("  Safe to declare [text, image] on this route. See docs/VISION-配置.md.")
        return
    print("\nVERDICT: the request succeeded but the answer does not describe the image.")
    print("  The endpoint accepted the field and most likely ignored it. Do NOT declare")
    print("  [text, image]: images would be admitted, committed to the session log, and")
    print("  then silently ignored, which is worse than a refusal.")
    raise SystemExit(1)


if __name__ == "__main__":
    main()
