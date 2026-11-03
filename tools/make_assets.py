#!/usr/bin/env python3
"""Generates assets/car.png, assets/charging_station.png and assets/background.png (needs Pillow)."""
import os
import random

from PIL import Image, ImageDraw, ImageFilter

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "assets")
os.makedirs(OUT, exist_ok=True)
S = 4  # supersampling factor for smooth edges


def car():
    w, h = 96, 48
    im = Image.new("RGBA", (w * S, h * S), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    def r(x0, y0, x1, y1, **kw):
        d.rounded_rectangle([x0 * S, y0 * S, x1 * S, y1 * S], **kw)
    # wheels
    for x in (16, 62):
        r(x, 1, x + 18, 8, radius=3 * S, fill=(20, 20, 24, 255))
        r(x, 40, x + 18, 47, radius=3 * S, fill=(20, 20, 24, 255))
    # shadow + body (facing right)
    r(4, 7, 92, 41, radius=14 * S, fill=(0, 0, 0, 70))
    r(3, 6, 91, 40, radius=14 * S, fill=(36, 132, 255, 255), outline=(10, 70, 160, 255), width=2 * S)
    # roof / glass
    r(30, 11, 70, 35, radius=7 * S, fill=(20, 40, 70, 255))
    r(55, 12, 69, 34, radius=5 * S, fill=(120, 200, 255, 255))     # windshield
    r(32, 12, 40, 34, radius=4 * S, fill=(90, 160, 220, 255))      # rear glass
    # lights
    r(86, 9, 91, 17, radius=2 * S, fill=(255, 245, 190, 255))
    r(86, 31, 91, 39, radius=2 * S, fill=(255, 245, 190, 255))
    r(3, 9, 7, 17, radius=2 * S, fill=(255, 70, 70, 255))
    r(3, 31, 7, 39, radius=2 * S, fill=(255, 70, 70, 255))
    # green EV leaf badge
    d.ellipse([44 * S, 20 * S, 52 * S, 28 * S], fill=(70, 220, 120, 255))
    im = im.resize((w, h), Image.LANCZOS)
    im.save(os.path.join(OUT, "car.png"))


def station():
    w = 64
    im = Image.new("RGBA", (w * S, w * S), (0, 0, 0, 0))
    d = ImageDraw.Draw(im)
    d.ellipse([3 * S, 3 * S, 61 * S, 61 * S], fill=(24, 160, 90, 255), outline=(255, 255, 255, 255), width=3 * S)
    bolt = [(36, 10), (20, 35), (30, 35), (26, 54), (45, 27), (34, 27)]
    d.polygon([(x * S, y * S) for x, y in bolt], fill=(255, 235, 80, 255))
    im = im.resize((w, w), Image.LANCZOS)
    im.save(os.path.join(OUT, "charging_station.png"))


def background():
    w, h = 580, 780
    rnd = random.Random(3)
    im = Image.new("RGB", (w, h))
    px = im.load()
    for y in range(h):
        for x in range(w):
            t = y / h
            base = (int(30 + 10 * t), int(40 + 12 * t), int(48 + 8 * t))
            n = rnd.randint(-3, 3)
            px[x, y] = (base[0] + n, base[1] + n, base[2] + n)
    im = im.filter(ImageFilter.GaussianBlur(1.2))
    d = ImageDraw.Draw(im)
    for x in range(0, w, 48):
        d.line([(x, 0), (x, h)], fill=(52, 66, 78), width=1)
    for y in range(0, h, 48):
        d.line([(0, y), (w, y)], fill=(52, 66, 78), width=1)
    im.save(os.path.join(OUT, "background.png"))


if __name__ == "__main__":
    car(); station(); background()
    print("assets written to", os.path.abspath(OUT))
