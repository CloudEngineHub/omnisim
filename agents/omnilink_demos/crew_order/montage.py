# Copyright 2026 OmniLink
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""The stability montage: 16 runs at once, fast-forwarded, then the tally.

    python agents/omnilink_demos/crew_order/montage.py <campaign_dir> <out.mp4> [--film draft.mp4 --final final.mp4]

Each tile is one campaign run's own `tile` camera, from one second before
Sam's order to a few seconds after that run's verdict, played at a constant
speed (exactly STEP captured frames per output frame). A tile's mark comes
from that run's result.json: a check only when every part was measured in the
crate AND the robots reported to Sam; anything else shows a cross. The closing
card prints summary.json and nothing else.
"""
import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from bisect import bisect_right
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

HERE = Path(__file__).resolve().parent
FPS = 24
STEP = 2                       # captured frames per output frame (256 ms frames -> 12.3x)
W, H = 1920, 1080
TW, TH = 480, 270
FONT = "C:/Windows/Fonts/segoeui.ttf"
FONT_BOLD = "C:/Windows/Fonts/segoeuib.ttf"
OK_RGB, BAD_RGB = (111, 231, 208), (255, 110, 90)


def font(n, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT, n)


def load_run(run: Path):
    rows = [json.loads(x) for x in (run / "crew.jsonl").read_text(encoding="utf-8").splitlines()]
    res = json.loads((run / "result.json").read_text(encoding="utf-8"))
    order_t = next(r["sim_s"] for r in rows if r["kind"] == "operator" and "Order" in r["text"])
    end_t = next((r["sim_s"] for r in rows if r["kind"] == "message" and r["to"] == "Sam"), None)
    final_t = next((r["sim_s"] for r in reversed(rows) if r["kind"] == "final" and "sim_s" in r), None)
    files = sorted((run / "frames" / "tile").glob("*.jpg"))
    stamps = [int(p.stem) / 1000.0 for p in files]
    i0 = max(0, bisect_right(stamps, order_t - 1.0) - 1)
    verdict_t = end_t if end_t is not None else (final_t or stamps[-1])
    i1 = min(len(files) - 1, bisect_right(stamps, verdict_t) - 1)
    return {"files": files, "stamps": stamps, "i0": i0, "i1": i1, "order_t": order_t, "res": res}


def tile(run, k, name):
    """Output frame k of one run's tile: playing, then holding its last frame with the verdict."""
    i = min(run["i0"] + k * STEP, run["i1"])
    done = run["i0"] + k * STEP >= run["i1"]
    img = None
    for j in range(i, -1, -1):            # a frame cut short by the engine stopping mid-write
        try:
            img = Image.open(run["files"][j]).convert("RGB").resize((TW, TH), Image.LANCZOS)
            break
        except OSError:
            continue
    d = ImageDraw.Draw(img, "RGBA")
    t = run["stamps"][i] - run["order_t"]
    d.rounded_rectangle((8, TH - 34, 150, TH - 8), radius=6, fill=(12, 20, 29, 210))
    d.text((16, TH - 33), f"{name}  {max(0, int(t)) // 60}:{max(0, int(t)) % 60:02d}", font=font(17, True),
           fill=(245, 247, 249, 255))
    if done:
        ok = bool(run["res"].get("ok"))
        col = OK_RGB if ok else BAD_RGB
        d.rectangle((0, 0, TW - 1, TH - 1), outline=(*col, 255), width=4)
        label = "✓ packed" if ok else "✗ " + ("not packed" if not run["res"].get("packed") else "no report")
        tw = d.textlength(label, font=font(22, True))
        d.rounded_rectangle((TW - tw - 32, 10, TW - 10, 46), radius=8, fill=(12, 20, 29, 225))
        d.text((TW - tw - 21, 12), label, font=font(22, True), fill=(*col, 255))
    return img, done


def card(summary, seconds, tmp, n0):
    n = n0
    lines = [
        (f"{summary['runs']} independent runs", 64, True, (245, 247, 249)),
        (f"{summary['ok']} packed and reported", 52, True, OK_RGB if summary["ok"] == summary["runs"] else BAD_RGB),
        (f"{summary['runs'] - summary['ok']} failed · {summary['crashed']} crashed · "
         f"{summary['delivery_errors']} message-delivery errors", 34, False, (200, 206, 212)),
        ("Each run: three robots, each its own OmniLink agent (Claude Opus 5.5), one order from Sam.", 26, False, (160, 168, 176)),
        ("Simulated in OmniSim. A run passes only if all three parts were measured in the crate and Sam got the report.", 26, False, (160, 168, 176)),
    ]
    img = Image.new("RGB", (W, H), (12, 20, 29))
    d = ImageDraw.Draw(img)
    y = 300
    for text, size, bold, col in lines:
        f = font(size, bold)
        d.text(((W - d.textlength(text, font=f)) / 2, y), text, font=f, fill=col)
        y += int(size * 1.6)
    d.text((40, 32), "OmniLink", font=font(30, True), fill=(245, 247, 249))
    for _ in range(int(seconds * FPS)):
        img.save(tmp / f"{n:06d}.png")
        n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("campaign")
    ap.add_argument("out")
    ap.add_argument("--film", default=None, help="the single-run film to put first")
    ap.add_argument("--final", default=None, help="write film + montage concatenated here")
    a = ap.parse_args()
    camp = Path(a.campaign)
    summary = json.loads((camp / "summary.json").read_text(encoding="utf-8"))
    runs = []
    for k in range(1, 17):
        run = camp / f"run_{k:03d}"
        if (run / "result.json").exists() and (run / "frames" / "tile").exists():
            runs.append((f"RUN {k:02d}", load_run(run)))
    tmp = Path(tempfile.mkdtemp(prefix="crew_montage_"))
    longest = max(r["i1"] - r["i0"] for _, r in runs) // STEP + 1
    n = 0
    for k in range(longest + FPS * 2):
        canvas = Image.new("RGB", (W, H), (12, 20, 29))
        for j, (name, run) in enumerate(runs):
            img, _ = tile(run, k, name)
            canvas.paste(img, ((j % 4) * TW, (j // 4) * TH))
        d = ImageDraw.Draw(canvas, "RGBA")
        banner = f"{len(runs)} of {summary['runs']} runs, side by side · each from its own order · {6.144 * STEP:.0f}× speed"
        f = font(26, True)
        tw = d.textlength(banner, font=f)
        d.rounded_rectangle(((W - tw) / 2 - 18, 14, (W + tw) / 2 + 18, 58), radius=10, fill=(12, 20, 29, 225))
        d.text(((W - tw) / 2, 18), banner, font=f, fill=(245, 247, 249, 255))
        canvas.save(tmp / f"{n:06d}.png")
        n += 1
    n = card(summary, 5.0, tmp, n)
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(FPS), "-i", str(tmp / "%06d.png"),
                    "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-an", a.out], check=True)
    shutil.rmtree(tmp, ignore_errors=True)
    print(f"{a.out}: {n / FPS:.1f} s, {len(runs)} tiles")
    if a.film and a.final:
        lst = Path(a.out).with_suffix(".concat.txt")
        lst.write_text(f"file '{Path(a.film).resolve().as_posix()}'\nfile '{Path(a.out).resolve().as_posix()}'\n", encoding="utf-8")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", str(lst),
                        "-c:v", "libx264", "-preset", "slow", "-crf", "18", "-pix_fmt", "yuv420p", "-an", a.final], check=True)
        print(f"{a.final}: film + montage")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    main()
