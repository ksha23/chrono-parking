"""The shape of the road surface: flat, or as uneven as a real parking lot is."""

import os
import tempfile

import numpy as np


class Ground:
    """The height of the road over the scenario. With `amp` > 0 it is not a plane: it rises and
    falls by up to `amp` metres, in waves between 6 and 25 m long, which is what paving and a few
    years of settling leave (no steps and no potholes). The car, the parked cars, the kerbs and
    the paint all sit on that surface. With amp = 0 the road is the plane z = 0."""

    STEP = 0.5            # spacing of the height samples [m]

    def __init__(self, bounds, amp=0.0, seed=0, margin=30.0):
        self.amp = amp
        self.x0, self.y0 = bounds[0] - margin, bounds[1] - margin
        self.nx = int(round((bounds[2] - bounds[0] + 2.0 * margin) / self.STEP)) + 1
        self.ny = int(round((bounds[3] - bounds[1] + 2.0 * margin) / self.STEP)) + 1
        self.h = np.zeros((self.ny, self.nx))
        if amp > 0.0:
            rng = np.random.default_rng(3000 + seed)
            fx, fy = np.fft.fftfreq(self.nx, self.STEP)[None, :], np.fft.fftfreq(self.ny, self.STEP)[:, None]
            f = np.hypot(fx, fy)
            band = (f > 1.0 / 25.0) & (f < 1.0 / 6.0)
            h = np.fft.ifft2(np.fft.fft2(rng.normal(size=(self.ny, self.nx))) * band).real
            self.h = amp * h / np.abs(h).max()

    @property
    def size(self):
        return (self.nx - 1) * self.STEP, (self.ny - 1) * self.STEP

    @property
    def center(self):
        return self.x0 + 0.5 * self.size[0], self.y0 + 0.5 * self.size[1]

    def height(self, x, y):
        """Height of the road at (x, y), between the samples by bilinear interpolation."""
        if self.amp == 0.0:
            return np.zeros(np.shape(x)) if np.ndim(x) else 0.0
        u = np.clip((np.asarray(x, dtype=float) - self.x0) / self.STEP, 0.0, self.nx - 1.001)
        v = np.clip((np.asarray(y, dtype=float) - self.y0) / self.STEP, 0.0, self.ny - 1.001)
        i, j = np.floor(u).astype(int), np.floor(v).astype(int)
        a, b = u - i, v - j
        h = self.h
        out = (1 - a) * (1 - b) * h[j, i] + a * (1 - b) * h[j, i + 1] + (1 - a) * b * h[j + 1, i] + a * b * h[j + 1, i + 1]
        return out if np.ndim(out) else float(out)

    def slope(self, x, y, reach=1.5):
        """Slope of the road around (x, y) along x and along y, over `reach` metres each way:
        about what the four wheels of a car standing there rest on."""
        return ((self.height(x + reach, y) - self.height(x - reach, y)) / (2.0 * reach),
                (self.height(x, y + reach) - self.height(x, y - reach)) / (2.0 * reach))

    def image(self, write_png):
        """The heights as a grey image for Chrono's height-map terrain (black lowest, white
        highest, first row at the largest y), and the two heights that black and white stand for."""
        lo, hi = float(self.h.min()), float(self.h.max())
        grey = np.round((self.h[::-1] - lo) / max(hi - lo, 1e-9) * 255.0).astype(np.uint8)
        self.h = lo + grey[::-1] / 255.0 * (hi - lo)            # exactly what the image holds
        path = os.path.join(tempfile.gettempdir(), "parking_ground_%d.png" % os.getpid())
        write_png(path, np.repeat(grey[..., None], 3, axis=2))
        return path, lo, hi
