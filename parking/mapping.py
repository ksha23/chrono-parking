"""Mapping: the occupancy grid built from the scans, and the line tracks built from the line detections."""

import math

import numpy as np

from .vehicle import EGO


class GridMap:
    RES = 0.1
    RANGE = 20.0           # longest ray any sensor contributes

    def __init__(self, bounds):
        self.x0, self.y0 = bounds[0], bounds[1]
        self.nx = int(math.ceil((bounds[2] - bounds[0]) / self.RES))
        self.ny = int(math.ceil((bounds[3] - bounds[1]) / self.RES))
        self.hits = np.zeros((self.ny, self.nx), dtype=np.uint16)
        self.free = np.zeros((self.ny, self.nx), dtype=np.uint16)
        # A camera sees ground further than it can tell ground from a kerb. That far ground is kept
        # apart: it lets the planner use the road ahead, but it is no evidence against an obstacle,
        # and a closer look that finds something there (stop) overrules it.
        self.far = np.zeros((self.ny, self.nx), dtype=np.uint16)
        self.stop = np.zeros((self.ny, self.nx), dtype=np.uint16)
        self._t = np.arange(0.1, self.RANGE, 0.2)

    def cells(self, x, y):
        ix = np.floor((np.asarray(x) - self.x0) / self.RES).astype(int)
        iy = np.floor((np.asarray(y) - self.y0) / self.RES).astype(int)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix, iy, ok

    def update(self, origin, ang, r_hit, r_free, r_far=None, r_stop=None):
        """One planar scan: an obstacle at range r_hit along each bearing (NaN: none), and free
        space along the bearing up to r_free. Optionally, ground that looks free but is too far
        to be sure, from r_free to r_far, and the range r_stop at which something that is not
        ground was seen. A scan counts once per cell, however many of its rays cross the cell."""
        def mark(counter, m, r=None):
            if r is None:           # all cells along the rays selected by m
                x = (origin[0] + self._t[None, :] * np.cos(ang)[:, None])[m]
                y = (origin[1] + self._t[None, :] * np.sin(ang)[:, None])[m]
            else:                   # the cell at range r of each ray selected by m
                x, y = origin[0] + r[m] * np.cos(ang[m]), origin[1] + r[m] * np.sin(ang[m])
            ix, iy, ok = self.cells(x, y)
            once = np.zeros_like(self.free, dtype=bool)
            once[iy[ok], ix[ok]] = True
            counter += once

        mark(self.hits, np.isfinite(r_hit), r_hit)
        mark(self.free, self._t[None, :] < r_free[:, None])
        if r_far is not None:
            mark(self.far, (self._t[None, :] >= r_free[:, None]) & (self._t[None, :] < r_far[:, None]))
        if r_stop is not None:
            mark(self.stop, np.isfinite(r_stop), r_stop)

    def mark_free(self, pose):
        """The ground the car stands on is free, whether a sensor looks at it or not."""
        x, y, th = pose
        win = self.window(x - EGO.length, y - EGO.length, x + EGO.length, y + EGO.length)
        if win is None:
            return
        X, Y, sl = win
        c, s = math.cos(th), math.sin(th)
        lon, lat = (X - x) * c + (Y - y) * s, -(X - x) * s + (Y - y) * c
        self.free[sl] += (lon > -EGO.rear) & (lon < EGO.front) & (np.abs(lat) < EGO.half_width)

    def occupied(self):
        # a real surface stops the rays; a cell they mostly pass through only caught range noise
        return (self.hits >= 2) & (self.hits > 0.3 * self.free)

    def blocked(self):
        """Planning map: cells with an obstacle in them, or that were never seen to be free
        (the far side of a parked car is unknown, not empty)."""
        known = self.grow(self.free >= 1, 2)       # 2 cells, to close the gaps between rays
        if self.far.any():
            # Far ground is located to a few tens of centimetres at best: use it only 0.4 m in
            # from where it ends, so that a plan keeps that distance from what is still unknown.
            maybe = self.grow((self.far >= 3) & (self.stop == 0), 2) & (self.stop == 0)
            known |= ~self.grow(~(known | maybe), 6)
        return self.occupied() | ~known

    @staticmethod
    def grow(mask, r):
        """The mask with every cell within r cells of it added (a square neighbourhood)."""
        for axis in (0, 1):
            pad = np.pad(mask, [(r, r) if a == axis else (0, 0) for a in (0, 1)])
            n = mask.shape[axis]
            mask = np.zeros_like(mask)
            for k in range(2 * r + 1):
                mask |= pad[k:k + n] if axis == 0 else pad[:, k:k + n]
        return mask

    def occupied_points(self):
        iy, ix = np.nonzero(self.occupied())
        return np.stack([self.x0 + (ix + 0.5) * self.RES, self.y0 + (iy + 0.5) * self.RES], axis=1)

    def window(self, xmin, ymin, xmax, ymax):
        """Cell-centre coordinates and index slices of the cells covering a box."""
        i0 = max(0, int((xmin - self.x0) / self.RES)); i1 = min(self.nx, int((xmax - self.x0) / self.RES) + 1)
        j0 = max(0, int((ymin - self.y0) / self.RES)); j1 = min(self.ny, int((ymax - self.y0) / self.RES) + 1)
        if i1 <= i0 or j1 <= j0:
            return None
        xs = self.x0 + (np.arange(i0, i1) + 0.5) * self.RES
        ys = self.y0 + (np.arange(j0, j1) + 0.5) * self.RES
        X, Y = np.meshgrid(xs, ys)
        return X, Y, (slice(j0, j1), slice(i0, i1))


class LineTrack:
    """One painted line, estimated from every detection associated with it.

    With `keep`, the track remembers along which stretch paint has been seen, also after the
    detections that showed it have been forgotten. A sensor with a limited view needs that: close
    to a stall the camera sees only the far part of its lines, and the part at the mouth, which it
    saw from further away, must not fade."""
    MAX_DETS = 160
    BIN = 0.1

    def __init__(self, det, t, keep=False):
        self.dets = [det]
        self.born = t
        self.c = np.zeros(2)
        self.d = np.array([1.0, 0.0])
        self.lo = self.hi = 0.0
        self.keep = keep
        self.anchor = 0.5 * (np.array(det[0:2]) + np.array(det[2:4]))
        self.seen = {}               # bin along the line (from the anchor) -> accumulated weight
        self.fit((det,))

    @property
    def hits(self):
        return len(self.dets)

    @property
    def length(self):
        return self.hi - self.lo

    def ends(self):
        return self.c + self.lo * self.d, self.c + self.hi * self.d

    def add(self, det):
        self.dets.append(det)
        if len(self.dets) > self.MAX_DETS:           # forget the least informative detection
            self.dets.pop(int(np.argmax([d[4] for d in self.dets])))
        self.fit((det,))

    def cover(self, det, d):
        """Record the stretch of the line that a detection shows."""
        a, b = sorted(((np.array(det[0:2]) - self.anchor) @ d, (np.array(det[2:4]) - self.anchor) @ d))
        w = 1.0 / (0.05 + 0.02 * det[4]) ** 2
        for k in range(int(math.floor(a / self.BIN)), int(math.floor(b / self.BIN)) + 1):
            self.seen[k] = self.seen.get(k, 0.0) + w

    def fit(self, fresh=()):
        """Weighted total least squares line + extent from the coverage of the detections.
        fresh: the detections that are new since the last fit."""
        D = np.asarray(self.dets)
        w = 1.0 / (0.05 + 0.02 * D[:, 4]) ** 2
        P = np.concatenate([D[:, 0:2], D[:, 2:4]])
        W = np.concatenate([w, w])
        c = (P * W[:, None]).sum(0) / W.sum()
        Q = P - c
        sxx, syy, sxy = (W * Q[:, 0] ** 2).sum(), (W * Q[:, 1] ** 2).sum(), (W * Q[:, 0] * Q[:, 1]).sum()
        th = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        d = np.array([math.cos(th), math.sin(th)])
        if d @ self.d < 0.0:
            d = -d
        a, b = Q[:len(D)] @ d, Q[len(D):] @ d
        a, b = np.minimum(a, b), np.maximum(a, b)
        if len(D) < 3:
            lo, hi = a.min(), b.max()
        else:
            edges = np.arange(a.min(), b.max() + 0.2, 0.2)
            mid = 0.5 * (edges[:-1] + edges[1:]) if len(edges) > 1 else np.array([0.5 * (a.min() + b.max())])
            cov = ((mid[None, :] >= a[:, None]) & (mid[None, :] <= b[:, None])) * w[:, None]
            cov = cov.sum(0)
            keep = np.flatnonzero(cov >= 0.25 * cov.max())
            lo, hi = mid[keep[0]] - 0.1, mid[keep[-1]] + 0.1
            # that extent overshoots (it follows the longest detections): take the weighted
            # median of the detection ends that lie close to each end of it instead
            for vals, ref, is_lo in ((a, lo, True), (b, hi, False)):
                near = np.abs(vals - ref) < 0.6
                if near.sum() >= 3:
                    order = np.argsort(vals[near])
                    cw = np.cumsum(w[near][order])
                    med = vals[near][order][np.searchsorted(cw, 0.5 * cw[-1])]
                    lo, hi = (med, hi) if is_lo else (lo, med)
        if self.keep:
            for det in fresh:
                self.cover(det, d)
            ks = np.array(sorted(self.seen))
            ws = np.array([self.seen[k] for k in ks])
            ks = ks[ws >= min(0.25 * ws.max(), 150.0)]
            off = (self.anchor - c) @ d
            k_lo, k_hi = off + ks[0] * self.BIN, off + (ks[-1] + 1) * self.BIN
            # the remembered stretch is known to the size of a bin: where the current detections
            # put an end close to it, theirs is the better value
            lo = lo if 0.0 <= lo - k_lo < 0.3 else min(lo, k_lo)
            hi = hi if 0.0 <= k_hi - hi < 0.3 else max(hi, k_hi)
        self.c, self.d, self.lo, self.hi = c, d, lo, hi


class LineMap:
    MIN_HITS = 5

    def __init__(self, keep=False):
        self.tracks = []
        self.keep = keep             # see LineTrack

    def confirmed(self):
        return [t for t in self.tracks if t.hits >= self.MIN_HITS and t.length > 1.2]

    def markers(self):
        """Confirmed lines plus the short stubs that are all a camera sees of a line between two cars."""
        return [t for t in self.tracks if t.hits >= self.MIN_HITS and t.length > 0.35]

    def update(self, dets, t):
        for det in dets:
            p1, p2 = np.array(det[0:2]), np.array(det[2:4])
            e = p2 - p1
            ln = np.hypot(*e)
            if ln < 0.3:
                continue
            e /= ln
            best, best_d = None, 1e9
            for trk in self.tracks:
                if min(ln, trk.length) > 1.0 and abs(e[0] * trk.d[1] - e[1] * trk.d[0]) > 0.21:
                    continue                    # (a short piece has no direction worth comparing)
                q1, q2 = p1 - trk.c, p2 - trk.c
                perp = max(abs(q1[0] * trk.d[1] - q1[1] * trk.d[0]), abs(q2[0] * trk.d[1] - q2[1] * trk.d[0]))
                if perp > 0.35 + 0.03 * det[4]:
                    continue
                s1, s2 = sorted((q1 @ trk.d, q2 @ trk.d))
                if max(trk.lo - s2, s1 - trk.hi) > 0.8:
                    continue
                if perp < best_d:
                    best, best_d = trk, perp
            if best is None:
                self.tracks.append(LineTrack(det, t, self.keep))
            else:
                best.add(det)
        self.tracks = [k for k in self.tracks if k.hits >= 3 or t - k.born < 2.5]
        self._merge()

    def _merge(self):
        i = 0
        while i < len(self.tracks):
            a = self.tracks[i]
            j = i + 1
            while j < len(self.tracks):
                b = self.tracks[j]
                q = b.c - a.c
                if (abs(a.d[0] * b.d[1] - a.d[1] * b.d[0]) < 0.1 and abs(q[0] * a.d[1] - q[1] * a.d[0]) < 0.25):
                    e1, e2 = b.ends()
                    s1, s2 = sorted(((e1 - a.c) @ a.d, (e2 - a.c) @ a.d))
                    if max(a.lo - s2, s1 - a.hi) < 0.4:
                        a.dets += b.dets
                        a.born = min(a.born, b.born)
                        for k, w in b.seen.items():          # what b remembers, in a's bins
                            ka = int(math.floor(((b.anchor + (k + 0.5) * b.BIN * b.d - a.anchor) @ a.d) / a.BIN))
                            a.seen[ka] = a.seen.get(ka, 0.0) + w
                        a.fit()
                        del self.tracks[j]
                        continue
                j += 1
            i += 1
