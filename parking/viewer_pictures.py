"""The part of the window that shows what the sensors deliver."""

import ctypes
import math

import numpy as np
import pychrono.irrlicht as irr

from .draw import RAMP, _IrrString, resample


class PicturesMixin:
    """What the sensors deliver, as pictures in the window. Part of Viewer."""

    # ---- what the sensors deliver ---------------------------------------------------

    @staticmethod
    def _texture_call():
        """Irrlicht's function that makes a texture from an image, or None if it cannot be reached.
        The Python bindings can draw a texture but cannot make one from pixels: the call for that,
        IVideoDriver::addTexture(name, image), takes an Irrlicht string, which they do not convert.
        So it is called through ctypes instead, by its C++ symbol (the same for clang and gcc)."""
        if not all(hasattr(irr, n) for n in ("ECF_A8R8G8B8", "dimension2du", "recti")):
            return None
        symbol = "_ZN3irr5video11CNullDriver10addTextureERKNS_4core6stringIcNS2_12irrAllocatorIcEEEEPNS0_6IImageEPv"
        libs = [None]                              # everything already in the process: enough on macOS
        try:
            with open("/proc/self/maps") as f:     # Linux: Irrlicht is loaded, but privately to PyChrono
                libs += sorted({ln.split()[-1] for ln in f if "libIrrlicht" in ln})
        except OSError:
            pass
        for lib in libs:
            try:
                call = ctypes.CDLL(lib)[symbol]
            except Exception:
                continue
            call.restype = ctypes.c_void_p
            call.argtypes = [ctypes.c_void_p] * 4
            return call
        return None

    def _upload(self, name, rgb, old=None):
        """A texture with the pixels of rgb (rows, columns, 3), replacing the texture 'old'."""
        h, w = rgb.shape[:2]
        bgra = np.empty((h, w, 4), np.uint8)
        bgra[..., 0], bgra[..., 1], bgra[..., 2], bgra[..., 3] = rgb[..., 2], rgb[..., 1], rgb[..., 0], 255
        img = self.drv.createImage(irr.ECF_A8R8G8B8, irr.dimension2du(w, h))
        ctypes.memmove(int(img.lock()), bgra.ctypes.data, bgra.nbytes)
        img.unlock()
        if old is not None:
            self.drv.removeTexture(old)
        raw = name.encode()
        text = _IrrString(raw, len(raw) + 1, len(raw) + 1, None)
        addr = self.add_texture(int(self.drv.this), ctypes.addressof(text), int(img.this), None)
        img.drop()
        for i in range(self.drv.getTextureCount()):       # the same texture, as an object of the bindings
            tex = self.drv.getTextureByIndex(i)
            if int(tex) == addr:
                return tex
        raise RuntimeError("Irrlicht did not keep the texture")

    def _pictures(self):
        """Draw the camera images and the range data, renewed whenever the sensors have delivered."""
        if not self.pics:
            return
        rig = self.sim.sensor
        new, self.pic_stamp = rig.tick != self.pic_stamp, rig.tick
        for (key, r, title, sub), (dst, src) in zip(self.pics, self.pic_rects):
            if new:
                img = self._picture(key, r[2] - r[0], r[3] - r[1])
                if img is not None:
                    self.canvas = (img, r[0], r[1])
                    self._picture_labels(key, r, title, sub)
                    self.canvas = None
                    self.tex[key] = self._upload("sensor %s %s" % key, img, self.tex.get(key))
            if key in self.tex:
                self.drv.draw2DImage(self.tex[key], dst, src)
            else:
                self._rect(irr.SColor(255, 14, 15, 18), *r)

    def _picture(self, key, w, h):
        name, kind = key
        if name == "range":
            return self._range_picture(w, h)
        cam = next(c for c in self.sim.sensor.cameras if c["label"] == name)
        if kind == "image":
            return resample(cam["image"][::-1], w, h) if "image" in cam else None     # (bottom row first)
        if "range" not in cam:
            return None
        r = cam["range"][::-1]
        img = RAMP[(255.0 * (1.0 - np.clip(r / self.DEPTH_SCALE, 0.0, 1.0))).astype(np.uint8)]
        img[r < 0.05] = (24, 26, 31)                                # the network's answer was not used here
        return resample(img, w, h)

    def _range_view(self, w, h):
        """Geometry of the range picture: pixels per metre, the pixel of the middle of the car, its
        x in the chassis frame, and the height of the part above the lidar's range image."""
        rig = self.sim.sensor
        hh = h - (2 * rig.LIDAR_H + 18 if rig.lidar is not None else 0)
        return min(w, hh) / (2.0 * self.RANGE_SPAN), 0.5 * w, 0.5 * hh, 0.5 * (rig.own[0] + rig.own[1]), hh

    def _range_picture(self, w, h):
        """Every range the sensors measured, as points seen from above in the frame of the car
        (forward is up), coloured by their height. Below it the range image of the lidar."""
        rig = self.sim.sensor
        s, cx, cy, xm, hh = self._range_view(w, h)
        px = lambda q: (cx - q[1] * s, cy - (q[0] - xm) * s)
        if self.range_bg is None:
            bg = np.full((h, w, 3), (14, 15, 18), np.uint8)
            a = np.linspace(0.0, 2.0 * math.pi, 720)
            for r in (5.0, 10.0, 15.0, 20.0):                       # range rings
                u, v = (cx + r * s * np.cos(a)).astype(int), (cy + r * s * np.sin(a)).astype(int)
                ok = (u >= 0) & (u < w) & (v >= 0) & (v < hh)
                bg[v[ok], u[ok]] = (46, 50, 60)
            for dev, half, reach in [(c, 0.5 * rig.CAM_HFOV, c["reach"]) for c in rig.cameras if "reach" in c] + \
                    ([(rig.lidar, 0.5 * rig.LIDAR_HFOV, rig.LIDAR_RANGE)] if rig.lidar is not None else []):
                yaw = math.atan2(*dev.get("R", np.eye(3))[1::-1, 0])
                for a in (yaw - half, yaw + half):                  # the edges of what each sensor looks at
                    d = np.array([math.cos(a), math.sin(a), 0.0])
                    self._stroke(bg[:hh], px(dev["pos"]), px(dev["pos"] + reach * d), (62, 74, 104))
            self.range_bg = bg
        img = self.range_bg.copy()

        def plot(cloud, size, gain, colour=None):
            if cloud is None or not len(cloud):
                return
            u = np.floor(cx - cloud[:, 1] * s).astype(int)
            v = np.floor(cy - (cloud[:, 0] - xm) * s).astype(int)
            ok = (u >= 0) & (u <= w - size) & (v >= 0) & (v <= hh - size)
            # the colour says how high a point is, and the highest points are drawn last
            t = (255.0 * np.sqrt(np.clip(cloud[ok, 2], 0.0, self.HEIGHT_SCALE) / self.HEIGHT_SCALE)).astype(np.uint8)
            order = np.argsort(t, kind="stable")
            u, v = u[ok][order], v[ok][order]
            if colour is None:
                colour = (RAMP * gain).astype(np.uint8)[t[order]]
            for du in range(size):
                for dv in range(size):
                    img[v + dv, u + du] = colour

        for cam in rig.cameras:                                    # depth from one camera is the least certain
            plot(cam.get("cloud"), 1, 0.45 if cam["role"] == "mono" else 0.7 if rig.lidar is not None else 1.0)
        if rig.lidar is not None:
            plot(rig.lidar.get("cloud"), 2, 1.0)
        x0, x1, hw = rig.own
        c = [px(q) for q in ((x1, hw), (x1, -hw), (x0, -hw), (x0, hw))]
        for k in range(4):
            self._stroke(img[:hh], c[k], c[(k + 1) % 4], (240, 242, 246))
        self._stroke(img[:hh], px((x1 - 0.9, hw)), px((x1, 0.0)), (240, 242, 246))       # the nose
        self._stroke(img[:hh], px((x1 - 0.9, -hw)), px((x1, 0.0)), (240, 242, 246))
        if rig.lidar is not None and "range" in rig.lidar:
            # one row per beam, the highest on top, and the left of the car on the left
            r = rig.lidar["range"][::-1, ::-1][:, np.arange(w) * rig.LIDAR_W // w]
            strip = RAMP[(255.0 * (1.0 - np.clip(r / rig.LIDAR_RANGE, 0.0, 1.0))).astype(np.uint8)]
            strip[r < 0.05] = (24, 26, 31)
            img[h - 2 * rig.LIDAR_H:] = np.repeat(strip, 2, axis=0)
        return img

    def _scale_bar(self, x, y, ramp, lo, hi):
        """A colour scale with its two ends named, on a dark backing."""
        n, bw = 24, 4
        self._rect(irr.SColor(170, 0, 0, 0), x - 5, y - 5, x + 6 * (len(lo) + len(hi)) + n * bw + 15, y + 12)
        self._text(lo, x, y, 1)
        x += 6 * len(lo) + 4
        for k in range(n):
            col = ramp[k * 255 // (n - 1)]
            self._rect(irr.SColor(255, int(col[0]), int(col[1]), int(col[2])), x + k * bw, y, x + (k + 1) * bw, y + 7)
        self._text(hi, x + n * bw + 5, y, 1)

    def _picture_labels(self, key, r, title, sub):
        rig = self.sim.sensor
        grey = (190, 200, 210)
        self._rect(irr.SColor(150, 0, 0, 0), r[0] + 8, r[1] + 8, r[0] + 20 + max(12 * len(title), 6 * len(sub)), r[1] + 41)
        self._text(title, r[0] + 14, r[1] + 11)
        self._text(sub, r[0] + 14, r[1] + 30, 1, rgb=grey)
        if key[1] == "range":
            self._scale_bar(r[0] + 14, r[3] - 17, RAMP[::-1], "0", "%.0f M" % self.DEPTH_SCALE)
        if key[0] == "range":
            s, cx, cy, _, hh = self._range_view(r[2] - r[0], r[3] - r[1])
            for ring in (5, 10):
                self._text("%d M" % ring, r[0] + int(cx) + 4, r[1] + int(cy - ring * s) + 3, 1, rgb=(120, 128, 142))
            self._scale_bar(r[0] + 14, r[1] + hh - 17, RAMP, "HEIGHT 0", "%.1f M" % self.HEIGHT_SCALE)
            if rig.lidar is not None:
                self._text("LIDAR RANGE IMAGE, %.0f DEG FORWARD, 0 TO %.0f M" % (math.degrees(rig.LIDAR_HFOV), rig.LIDAR_RANGE),
                           r[0] + 14, r[1] + hh + 6, 1, rgb=grey)
