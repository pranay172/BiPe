"""Edit rendered demo shots into one video with ffmpeg: cards, a 2x2 grid, the cut.

python -m bipe.demo.edit card STILL.png OUT.mp4 --title BiPe --subtitle ... --seconds 4
python -m bipe.demo.edit grid A.mp4 B.mp4 C.mp4 D.mp4 OUT.mp4
python -m bipe.demo.edit cut OUT.mp4 SHOT.mp4 ... [--fade 0.4]
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from pathlib import Path

from .render import FONT

SIZE = (1280, 720)
FPS = 50


def _ffmpeg(*args):
    exe = shutil.which("ffmpeg")
    if exe is None:  # imageio's bundled binary (no system ffmpeg on the laptop)
        import imageio_ffmpeg

        exe = imageio_ffmpeg.get_ffmpeg_exe()
    subprocess.run([exe, "-y", "-v", "error", *map(str, args)], check=True)


def duration(path):
    if shutil.which("ffprobe") is None:
        import imageio_ffmpeg

        return float(imageio_ffmpeg.count_frames_and_secs(str(path))[1])
    out = subprocess.run(
        ["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "json", str(path)],
        check=True, capture_output=True, text=True,
    ).stdout  # fmt: skip
    return float(json.loads(out)["format"]["duration"])


def card(still, out, title, subtitle=None, note=None, seconds=4.0, side="left", window=None):
    """A title card: the still (cropped to 16:9) with text, faded in and out."""
    from PIL import Image, ImageDraw, ImageFont

    img = Image.open(still).convert("RGB")
    if window:  # (left, width) as fractions of the still: a zoomed, off-centre crop
        x0, wf = window
        W, H = img.size
        cw, ch = int(W * wf), int(W * wf * SIZE[1] / SIZE[0])
        top = (H - ch) // 2
        img = img.crop((int(W * x0), top, int(W * x0) + cw, top + ch))
    w, h = img.size
    target = SIZE[0] / SIZE[1]
    if w / h > target:
        nw = int(h * target)
        img = img.crop(((w - nw) // 2, 0, (w - nw) // 2 + nw, h))
    else:
        nh = int(w / target)
        img = img.crop((0, (h - nh) // 2, w, (h - nh) // 2 + nh))
    img = img.resize(SIZE, Image.LANCZOS)
    # A soft light panel behind the text side, fading out by mid-frame.
    shade = Image.new("L", SIZE, 0)
    for xx in range(SIZE[0]):
        u = xx / SIZE[0] if side == "left" else 1 - xx / SIZE[0]
        shade.paste(int(200 * max(0.0, 1 - u / 0.55) ** 1.5), (xx, 0, xx + 1, SIZE[1]))
    img = Image.composite(Image.new("RGB", SIZE, (236, 237, 240)), img, shade)
    g = ImageDraw.Draw(img)
    big, mid, small = (ImageFont.truetype(FONT, s) for s in (84 if len(title) <= 8 else 58, 30, 20))
    x = 72 if side == "left" else SIZE[0] - 72
    anchor = "ls" if side == "left" else "rs"
    g.text((x, 300), title, font=big, fill=(22, 24, 28), anchor=anchor)
    if subtitle:
        g.text((x, 350), subtitle, font=mid, fill=(50, 54, 62), anchor=anchor)
    if note:
        g.text((x, SIZE[1] - 48), note, font=small, fill=(70, 74, 82), anchor=anchor)
    png = Path(out).with_suffix(".png")
    img.save(png)
    fade = 0.5
    _ffmpeg(
        "-loop", "1", "-framerate", FPS, "-t", seconds, "-i", png,
        "-vf", f"fade=t=in:st=0:d={fade},fade=t=out:st={seconds - fade}:d={fade},format=yuv420p",
        "-c:v", "libx264", "-crf", 18, out,
    )  # fmt: skip
    return out


def grid(clips, out):
    """Four clips in a 2x2 grid at the cut's size."""
    w, h = SIZE[0] // 2, SIZE[1] // 2
    inputs = [a for c in clips for a in ("-i", c)]
    scale = ";".join(f"[{i}]scale={w}:{h},setsar=1[v{i}]" for i in range(4))
    layout = f"0_0|{w}_0|0_{h}|{w}_{h}"
    _ffmpeg(
        *inputs, "-filter_complex",
        f"{scale};[v0][v1][v2][v3]xstack=inputs=4:layout={layout}:shortest=1,format=yuv420p",
        "-r", FPS, "-c:v", "libx264", "-crf", 18, out,
    )  # fmt: skip
    return out


def pair(left, right, out):
    """Two clips side by side at the cut's size (each half width, letterboxed). The
    shorter clip holds its last frame (a run that ends at a fall stops recording)."""
    w, h = SIZE[0] // 2, SIZE[1]
    longest = max(duration(left), duration(right))
    fit = (
        f"scale={w}:-2,pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:color=0xd0d4dc,setsar=1,"
        f"tpad=stop_mode=clone:stop_duration={longest:.3f}"
    )
    _ffmpeg(
        "-i", left, "-i", right, "-filter_complex",
        f"[0]{fit}[a];[1]{fit}[b];[a][b]hstack=inputs=2:shortest=1,format=yuv420p",
        "-t", f"{longest:.3f}",
        "-r", FPS, "-c:v", "libx264", "-crf", 18, out,
    )  # fmt: skip
    return out


def cut(out, clips, fade=0.4):
    """Clips joined with crossfades, scaled to the cut's size and frame rate."""
    lengths = [duration(c) for c in clips]
    inputs = [a for c in clips for a in ("-i", c)]
    norm = ";".join(
        f"[{i}]scale={SIZE[0]}:{SIZE[1]}:force_original_aspect_ratio=decrease,"
        f"pad={SIZE[0]}:{SIZE[1]}:(ow-iw)/2:(oh-ih)/2,setsar=1,fps={FPS},format=yuv420p[n{i}]"
        for i in range(len(clips))
    )
    chain, last, offset = [], "n0", 0.0
    for i in range(1, len(clips)):
        offset += lengths[i - 1] - fade
        tag = f"x{i}"
        chain.append(
            f"[{last}][n{i}]xfade=transition=fade:duration={fade}:offset={offset:.3f}[{tag}]"
        )
        last = tag
    graph = ";".join([norm, *chain]) if chain else norm
    _ffmpeg(
        *inputs, "-filter_complex", graph, "-map", f"[{last}]",
        "-c:v", "libx264", "-crf", 18, "-pix_fmt", "yuv420p", "-movflags", "+faststart", out,
    )  # fmt: skip
    return out


def main():
    ap = argparse.ArgumentParser(prog="python -m bipe.demo.edit")
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("card")
    c.add_argument("still")
    c.add_argument("out")
    c.add_argument("--title", required=True)
    c.add_argument("--subtitle")
    c.add_argument("--note")
    c.add_argument("--seconds", type=float, default=4.0)
    c.add_argument("--side", default="left", choices=("left", "right"))
    c.add_argument("--window", type=float, nargs=2, metavar=("LEFT", "WIDTH"))
    g = sub.add_parser("grid")
    g.add_argument("clips", nargs=4)
    g.add_argument("out")
    q = sub.add_parser("pair")
    q.add_argument("left")
    q.add_argument("right")
    q.add_argument("out")
    k = sub.add_parser("cut")
    k.add_argument("out")
    k.add_argument("clips", nargs="+")
    k.add_argument("--fade", type=float, default=0.4)
    a = ap.parse_args()
    if a.cmd == "card":
        print(card(a.still, a.out, a.title, a.subtitle, a.note, a.seconds, a.side, a.window))
    elif a.cmd == "grid":
        print(grid(a.clips, a.out))
    elif a.cmd == "pair":
        print(pair(a.left, a.right, a.out))
    else:
        print(cut(a.out, a.clips, a.fade))


if __name__ == "__main__":
    main()
