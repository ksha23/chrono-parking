"""Painted lines as they are found on a real lot: never perfect.

Paint that has been driven over and rained on for years is patchy, with the road showing through,
its edges are ragged, and some lines have nearly gone. A line also wanders a little and changes
width, because it was laid by a machine that somebody pushed. This module makes the images for
that and decides how each piece of each line looks."""

import math
import os
import tempfile

import numpy as np

PIECE = 0.5                    # paint is laid down in pieces of this length [m]
WIDTH = 0.12                   # width of a new line [m]
TEXELS = (32, 128)             # an image is this many texels across and along a piece (4 mm each)
LEVELS, VARIANTS = 3, 3

# How worn a line is: share of the lines, reflectance of the paint (new paint is 0.8), share of a
# piece that is still covered, how far the edges are eaten away [m], share of the pieces that are
# gone altogether. No line is as it was painted.
STATES = (("worn", 2.0 / 3.0, (0.42, 0.65), 0.88, 0.008, 0.05),
          ("faded", 1.0 / 4.0, (0.32, 0.48), 0.78, 0.012, 0.10),
          ("nearly gone", 1.0 / 12.0, (0.22, 0.32), 0.55, 0.018, 0.20))


def _srgb(lin):
    lin = np.clip(lin, 0.0, 1.0)
    return (255.0 * np.where(lin <= 0.0031308, 12.92 * lin, 1.055 * lin ** (1.0 / 2.4) - 0.055) + 0.5).astype(np.uint8)


def paint_textures(write_png):
    """Image files of worn paint, keyed (colour, state, level, variant). They are made once.
    Where the paint is gone an image shows the grey of the road."""
    folder = os.path.join(tempfile.gettempdir(), "chrono_parking_paint_3")
    files = {(c, s, lv, v): os.path.join(folder, "%s_%d_%d_%d.png" % (c, s, lv, v))
             for c in ("white", "yellow") for s in range(len(STATES)) for lv in range(LEVELS) for v in range(VARIANTS)}
    if all(os.path.exists(f) for f in files.values()):
        return files
    os.makedirs(folder, exist_ok=True)
    rng = np.random.default_rng(11)
    h, w = TEXELS

    def noise(lo, hi, shape=TEXELS):       # wavelengths lo to hi texels, unit variance
        fy, fx = np.fft.fftfreq(shape[0])[:, None], np.fft.fftfreq(shape[1])[None, :]
        r = np.hypot(fy, fx)
        x = np.fft.ifft2((rng.normal(size=shape) + 1j * rng.normal(size=shape)) * ((r >= 1.0 / hi) & (r <= 1.0 / lo))).real
        return (x - x.mean()) / max(x.std(), 1e-9)

    across = (np.minimum(np.arange(h), h - 1 - np.arange(h)) + 0.5)[:, None] * (WIDTH / h)      # distance from the nearer edge [m]
    for (colour, state, level, variant), path in files.items():
        _, _, (lo, hi), cover, eaten, _ = STATES[state]
        paint = lo + (hi - lo) * (level + 0.5) / LEVELS
        paint = paint * (1.0 + 0.08 * noise(8.0, 60.0))                         # thicker and thinner coats
        holes = noise(6.0, 40.0) + 0.3 * noise(2.5, 6.0)                        # flaked patches of 2 to 16 cm, a few nicks
        there = holes > np.quantile(holes, 1.0 - cover)
        ragged = [eaten * (1.0 + noise(5.0, 50.0, (1, w)) + 0.4 * noise(2.0, 5.0, (1, w))) for _ in range(2)]      # one edge per side
        there &= across > np.concatenate([np.repeat(ragged[0], h // 2, axis=0), np.repeat(ragged[1], h - h // 2, axis=0)])
        road = 0.16 + 0.02 * noise(1.5, 6.0)
        r = np.where(there, paint, road)
        rgb = (r, r, r) if colour == "white" else (np.where(there, r, road), np.where(there, 0.80 * r, road), np.where(there, 0.10 * r, road))
        write_png(path, np.stack([_srgb(c) for c in rgb], axis=-1))
    return files


def lay(lines, wear, seed):
    """The pieces of every line of a scenario, as (x, y, yaw, length, width, colour, state, level,
    variant). Every line is worn. `wear` scales how many are worse than that: at 1, one in four
    is faded and one in twelve nearly gone."""
    rng = np.random.default_rng(2000 + seed)
    out = []
    for x1, y1, x2, y2, colour in lines:
        length, yaw = math.hypot(x2 - x1, y2 - y1), math.atan2(y2 - y1, x2 - x1)
        n = max(1, int(round(length / PIECE)))
        u = rng.random()
        state = 2 if u < wear * STATES[2][1] else 1 if u < wear * (STATES[2][1] + STATES[1][1]) else 0
        # the line wanders by up to a centimetre and is laid 85 to 100 percent wide
        swing, wave, phase = rng.uniform(0.003, 0.010), rng.uniform(2.5, 6.0), rng.uniform(0.0, 2.0 * math.pi)
        wide, wave_w, phase_w = rng.uniform(0.87, 0.97), rng.uniform(3.0, 8.0), rng.uniform(0.0, 2.0 * math.pi)
        for k in range(n):
            s = (k + 0.5) / n * length
            if rng.random() < STATES[state][5]:
                continue
            here = min(state + (rng.random() < 0.15), len(STATES) - 1)      # (a line is not worn evenly along its length)
            a = 2.0 * math.pi * s / wave + phase
            off, turn = swing * math.sin(a), math.atan(swing * 2.0 * math.pi / wave * math.cos(a))
            out.append((x1 + s * math.cos(yaw) - off * math.sin(yaw), y1 + s * math.sin(yaw) + off * math.cos(yaw), yaw + turn,
                        length / n + 0.004, WIDTH * wide * (1.0 + 0.06 * math.sin(2.0 * math.pi * s / wave_w + phase_w)),
                        colour, here, int(rng.integers(LEVELS)), int(rng.integers(VARIANTS))))
    return out
