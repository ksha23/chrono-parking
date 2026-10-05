"""The internals panel of the window."""

import math
import time

import numpy as np
import pychrono.irrlicht as irr

from .geometry import ego_poly
from .stalls import Slot
from .vehicle import EGO


class PanelMixin:
    """The internals panel. Part of Viewer."""

    # ---- internals panel -------------------------------------------------------
    # Everything here is drawn with filled rectangles, the one 2D primitive the Python bindings
    # have on every platform: pixel rasters are run-length encoded into rectangles. Where a
    # picture can be uploaded (see _texture_call), the rectangles go into an image that is renewed
    # at the perception rate and drawn as one texture, which costs a fraction of the time.

    PALETTE = {0: (62, 68, 80), 1: (22, 23, 27), 2: (235, 80, 60), 3: (52, 132, 160), 4: (130, 130, 130),
               5: (90, 200, 110), 6: (60, 255, 90), 7: (90, 150, 255), 8: (255, 110, 235), 9: (255, 240, 60),
               10: (240, 242, 246)}       # free, unseen, obstacle, searched, stall, free stall, target, fwd, rev, MPC, car

    @staticmethod
    def _stroke(img, a, b, value):
        n = int(max(abs(b[0] - a[0]), abs(b[1] - a[1]))) + 1
        xs = np.linspace(a[0], b[0], n).astype(int)
        ys = np.linspace(a[1], b[1], n).astype(int)
        ok = (xs >= 0) & (xs < img.shape[1]) & (ys >= 0) & (ys < img.shape[0])
        img[ys[ok], xs[ok]] = value

    def _map_image(self, w, h):
        """Planning map as a small palette image: unseen space, obstacles, the search tree."""
        sim, g = self.sim, self.sim.grid
        scale = min(w / (g.nx * g.RES), h / (g.ny * g.RES))          # pixels per metre
        pw, ph = int(g.nx * g.RES * scale), int(g.ny * g.RES * scale)
        ix = np.minimum((np.arange(pw) / scale / g.RES).astype(int), g.nx - 1)
        iy = np.minimum(((ph - 1 - np.arange(ph)) / scale / g.RES).astype(int), g.ny - 1)
        occ = g.occupied()
        fat = occ.copy()                                              # keep thin walls visible
        fat[1:, :] |= occ[:-1, :]
        fat[:, 1:] |= occ[:, :-1]
        img = np.where(fat[np.ix_(iy, ix)], 2, np.where(g.blocked()[np.ix_(iy, ix)], 1, 0)).astype(np.uint8)
        if sim.plan_info is not None and len(sim.plan_info["explored"]):
            e = sim.plan_info["explored"]
            ex = ((e[:, 0] - g.x0) * scale).astype(int)
            ey = ph - 1 - ((e[:, 1] - g.y0) * scale).astype(int)
            ok = (ex >= 0) & (ex < pw) & (ey >= 0) & (ey < ph)
            free = img[ey[ok], ex[ok]] == 0
            img[ey[ok][free], ex[ok][free]] = 3
        self.map_scale = scale
        return img

    def _map_rects(self, x, y):
        """The map with the stalls, plan, MPC horizon and car drawn in, as rectangles."""
        sim, trk, g = self.sim, self.sim.tracker, self.sim.grid
        img = self.map_img.copy()
        ph, scale = img.shape[0], self.map_scale
        px = lambda q: ((q[0] - g.x0) * scale, ph - 1 - (q[1] - g.y0) * scale)
        for sl in sim.slots:
            c = [px(q) for q in sl.corners]
            val = 6 if sl is sim.target else (5 if sl.status == Slot.FREE else 4)
            for k in range(4):
                self._stroke(img, c[k], c[(k + 1) % 4], val)
        if sim.state in ("DRIVE", "BRAKE", "PLAN"):
            for seg in sim.path[sim.seg_i:]:
                pts = [px((seg.x[k], seg.y[k])) for k in range(0, len(seg.x), 4)] + [px((seg.x[-1], seg.y[-1]))]
                for k in range(len(pts) - 1):
                    self._stroke(img, pts[k], pts[k + 1], 7 if seg.dir > 0 else 8)
        if sim.state in ("DRIVE", "SEARCH") and len(trk.horizon) > 1:
            pts = [px(q) for q in trk.horizon[::2]]
            for k in range(len(pts) - 1):
                self._stroke(img, pts[k], pts[k + 1], 9)
        c = [px(q) for q in ego_poly(sim.pose)]
        for k in range(4):
            self._stroke(img, c[k], c[(k + 1) % 4], 10)
        colors = {k: irr.SColor(255, *rgb) for k, rgb in self.PALETTE.items()}
        rects = []
        for row in range(ph):
            line = img[row]
            cuts = np.flatnonzero(np.diff(line)) + 1
            for a, b in zip(np.concatenate([[0], cuts]), np.concatenate([cuts, [len(line)]])):
                rects.append((colors[int(line[a])], x + int(a), y + row, x + int(b), y + row + 1))
        return rects

    def _trace(self, out, x, y, w, h, vals, lo, hi, rgb, slots=None):
        """Queue one strip-chart trace as a chain of small rectangles."""
        v = np.asarray(vals, dtype=float)
        if len(v) < 2:
            return
        px = (x + np.arange(len(v) + 1) * w / (slots or self.hist.maxlen)).astype(int)
        py = (y + h - 2 - (np.clip(v, lo, hi) - lo) / (hi - lo) * (h - 3)).astype(int)
        col = irr.SColor(255, *rgb)
        for k in range(len(v) - 1):
            out.append((col, px[k], min(py[k], py[k + 1]), max(px[k + 1], px[k] + 1), max(py[k], py[k + 1]) + 2))

    def _panel(self):
        if self.add_texture is None:
            return self._panel_draw()
        sim, x0 = self.sim, self.W - self.PW
        key = (self.hist_step, sim.state, sim.plan_info is None, len(sim.slots))
        if key != self.panel_key or time.time() - self.panel_wall > 0.25:
            self.panel_key, self.panel_wall = key, time.time()
            self.canvas = (np.empty((self.H - 30, self.PW, 3), np.uint8), x0, 30)
            self._panel_draw()
            img, self.canvas = self.canvas[0], None
            self.tex["panel"] = self._upload("panel", img, self.tex.get("panel"))
        self.drv.draw2DImage(self.tex["panel"], irr.recti(x0, 30, self.W, self.H), irr.recti(0, 0, self.PW, self.H - 30))

    def _panel_draw(self):
        sim, trk = self.sim, self.sim.tracker
        x, w = self.W - self.PW + 10, self.PW - 20
        self._rect(irr.SColor(255, 24, 26, 31), self.W - self.PW, 30, self.W, self.H)
        grey, white = (150, 158, 170), (235, 238, 242)
        y = 40

        # which stage of the pipeline is doing the work right now
        stages = ("SENSE", "MAP", "DECIDE", "PLAN", "TRACK")
        active = {"WAIT": (0, 1), "SEARCH": (0, 1, 4) if sim.manual is not None else (0, 1, 2, 4),
                  "BRAKE": (0, 1), "PLAN": (3,), "DRIVE": (0, 1, 4)}.get(sim.state, ())
        bw = (w - 16) // 5
        for k, name in enumerate(stages):
            on = k in active
            self._rect(irr.SColor(255, 40, 150, 90) if on else irr.SColor(255, 48, 52, 60),
                       x + k * (bw + 4), y, x + k * (bw + 4) + bw, y + 20)
            self._text(name, x + k * (bw + 4) + (bw - 6 * len(name)) // 2, y + 7, 1, rgb=white if on else grey)
        y += 32

        # layout: map, then four charts
        info = sim.plan_info
        self._text("PLANNING MAP", x, y, 1, rgb=grey)
        if info is not None:
            self._text("%d EXPANSIONS  COST %.0f" % (info["iterations"], info["cost"]), x + 150, y, 1, rgb=grey)
        map_y, map_h = y + 12, 132
        self._rect(irr.SColor(255, 14, 15, 18), x, map_y, x + w, map_y + map_h)
        y = map_y + map_h + 10
        tops = []
        for title, hgt in (("MPC HORIZON (CURVATURE OVER 4 M)", 52), ("TRACKING ERROR", 56), ("SPEED", 48), ("STEERING GAIN", 48)):
            self._text(title, x, y, 1, rgb=grey)
            self._rect(irr.SColor(255, 14, 15, 18), x, y + 12, x + w, y + 12 + hgt)
            self._rect(irr.SColor(255, 44, 48, 56), x, y + 12 + hgt // 2, x + w, y + 13 + hgt // 2)
            tops.append((y + 12, hgt))
            y += hgt + 22

        # the dynamic content is rebuilt at the perception rate and replayed in between
        if sim.time - self.map_time > 0.5 or self.map_img is None:
            self.map_time, self.map_img = sim.time, self._map_image(w, map_h)
            self.panel_rects = None
        k_max = math.tan(EGO.steer_max) * (trk.gain.g[trk.seg.dir] if trk.seg is not None else 1.0 / EGO.wheelbase)
        g_ideal = 1.0 / EGO.wheelbase
        if self.panel_rects is None:
            out = self._map_rects(x, map_y)
            hz = np.array(self.hist) if self.hist else np.zeros((0, 7))
            if sim.state in ("DRIVE", "SEARCH") and len(trk.k_plan):
                lim, n = 1.25 * max(k_max, EGO.kappa), len(trk.k_plan)
                ty, th = tops[0]
                for vals, rgb, thick in ((np.full(n, k_max), (150, 60, 60), 1), (np.full(n, -k_max), (150, 60, 60), 1),
                                         (trk.k_ref, (150, 158, 170), 1), (trk.k_plan, (255, 240, 60), 2)):
                    py = (ty + th - 2 - (np.clip(vals, -lim, lim) + lim) / (2 * lim) * (th - 3)).astype(int)
                    for k in range(n):
                        out.append((irr.SColor(255, *rgb), x + k * w // n, py[k], x + (k + 1) * w // n, py[k] + thick))
            if len(hz):
                self._trace(out, x, tops[1][0], w, tops[1][1], 100.0 * hz[:, 0], -12.0, 12.0, (90, 220, 255))
                self._trace(out, x, tops[1][0], w, tops[1][1], np.degrees(hz[:, 1]), -6.0, 6.0, (255, 170, 60))
                self._trace(out, x, tops[2][0], w, tops[2][1], hz[:, 3], -2.6, 2.6, (120, 126, 138))
                self._trace(out, x, tops[2][0], w, tops[2][1], hz[:, 2], -2.6, 2.6, (120, 255, 140))
                self._trace(out, x, tops[3][0], w, tops[3][1], np.full(len(hz), g_ideal), 0.0, 1.5 * g_ideal, (120, 126, 138))
                self._trace(out, x, tops[3][0], w, tops[3][1], hz[:, 5], 0.0, 1.5 * g_ideal, (90, 150, 255))
                self._trace(out, x, tops[3][0], w, tops[3][1], hz[:, 6], 0.0, 1.5 * g_ideal, (255, 110, 235))
            self.panel_rects = out
        for col, x0, y0, x1, y1 in self.panel_rects:
            self._rect(col, x0, y0, x1, y1)

        e, psi = (trk.err[0], trk.err[1]) if sim.state in ("DRIVE", "SEARCH") else (0.0, 0.0)
        self._text("PLAN", x + w - 138, tops[0][0] - 12, 1, rgb=(255, 240, 60))
        self._text("PATH", x + w - 104, tops[0][0] - 12, 1, rgb=grey)
        self._text("QP %2d IT" % trk.mpc.iters, x + w - 60, tops[0][0] - 12, 1, rgb=grey)
        self._text("E %+5.1f CM" % (100.0 * e), x + w - 170, tops[1][0] - 12, 1, rgb=(90, 220, 255))
        self._text("PSI %+4.1f DEG" % math.degrees(psi), x + w - 84, tops[1][0] - 12, 1, rgb=(255, 170, 60))
        self._text("%+.2f M/S" % sim.speed, x + w - 60, tops[2][0] - 12, 1, rgb=(120, 255, 140))
        self._text("FWD %.3f" % trk.gain.g[1], x + w - 184, tops[3][0] - 12, 1, rgb=(90, 150, 255))
        self._text("REV %.3f" % trk.gain.g[-1], x + w - 120, tops[3][0] - 12, 1, rgb=(255, 110, 235))
        self._text("1/L %.3f" % g_ideal, x + w - 56, tops[3][0] - 12, 1, rgb=grey)

        free = sum(sl.status == Slot.FREE for sl in sim.slots)
        occ = sum(sl.status == Slot.OCCUPIED for sl in sim.slots)
        rows = ["PERCEPTION: " + sim.sensor.name.upper().replace("::", " "),
                "LINE TRACKS %d (%d CONFIRMED)" % (len(sim.lines.tracks), len(sim.lines.confirmed())),
                "STALLS %d: %d FREE, %d OCCUPIED" % (len(sim.slots), free, occ),
                "CAR %.2f X %.2f M, WHEELBASE %.2f M" % (EGO.length, 2.0 * EGO.half_width, EGO.wheelbase),
                "%.0f KG, STEERING +-%.1f DEG, BRAKES %.0f NM" % (EGO.mass, math.degrees(EGO.steer_max), EGO.brake_torque_max),
                "MIN CLEARANCE SO FAR %.2f M" % (sim.min_clearance if sim.min_clearance < 1e9 else 0.0)]
        if info is not None:
            rows.append("PLAN MARGIN %.2f M, %d SEGMENTS" % (info["margin"], len(sim.path)))
        for k, row in enumerate(rows):
            if y + 12 * k + 10 < self.H:
                self._text(row, x, y + 12 * k, 1, rgb=grey)
        y += 12 * len(rows) + 10
        cams = [c for c in getattr(sim.sensor, "cameras", ()) if "view" in c]
        if cams and y + 12 + cams[0]["view"].shape[0] + 14 < self.H:
            stamp = tuple(c["stamp"] for c in cams)
            if stamp != self.cam_stamp:          # new images: turn each row into runs of one colour
                self.cam_stamp, self.cam_rects = stamp, []
                for k, c in enumerate(cams):
                    img = c["view"]
                    x0 = x + k * (img.shape[1] + 14)
                    for row in range(img.shape[0]):
                        cuts = np.flatnonzero(np.diff(img[row])) + 1
                        for a, b in zip(np.concatenate([[0], cuts]), np.concatenate([cuts, [img.shape[1]]])):
                            self.cam_rects.append((irr.SColor(255, *self.CAM_PALETTE[int(img[row, a])]),
                                                   x0 + int(a), y + 12 + row, x0 + int(b), y + 13 + row))
            for k, c in enumerate(cams):
                self._text(c["label"].upper() + ", AS READ", x + k * (c["view"].shape[1] + 14), y, 1, rgb=grey)
            for col, x0, y0, x1, y1 in self.cam_rects:
                self._rect(col, x0, y0, x1, y1)
            y += 12 + cams[0]["view"].shape[0] + 6
            lx = x
            for name, key in (("GROUND", 1), ("OBSTACLE", 2), ("UNCLEAR", 4), ("PAINT", 5), ("OWN BODY", 3)):
                self._rect(irr.SColor(255, *self.CAM_PALETTE[key]), lx, y, lx + 8, y + 8)
                self._text(name, lx + 11, y + 1, 1, rgb=grey)
                lx += 17 + 6 * len(name)
