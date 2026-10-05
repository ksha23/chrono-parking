"""Perception shared by both sources: the stand-in without sensors, and the geometry that turns
points and paint into planar scans and line segments."""

import math

import numpy as np

from .vehicle import EGO


class Perception:
    """Stand-in for a perception stack: no sensor is simulated, the detections are computed
    from the scenario and corrupted with noise, dropouts, fragmentation and clutter."""
    N_RAYS = 360
    SCAN_RANGE = 16.0
    LINE_RANGE = 12.0
    name = "simulated detections"

    def __init__(self, scn, noise, rng):
        self.noise = noise
        self.rng = rng
        edges = []
        for poly in scn.obstacle_polys():
            for i in range(len(poly)):
                edges.append((poly[i][0], poly[i][1], poly[(i + 1) % len(poly)][0], poly[(i + 1) % len(poly)][1]))
        self.edges = np.array(edges).reshape(-1, 4)
        self.line_pts = []
        for x1, y1, x2, y2, _ in scn.lines:
            n = max(2, int(math.hypot(x2 - x1, y2 - y1) / 0.25) + 1)
            self.line_pts.append(np.linspace((x1, y1), (x2, y2), n))

    def sense(self, pose):
        """Returns (scans, line detections) for the car at this rear-axle pose."""
        rng, k = self.rng, self.noise
        ox = pose[0] + EGO.center * math.cos(pose[2])
        oy = pose[1] + EGO.center * math.sin(pose[2])
        ang = pose[2] + np.arange(self.N_RAYS) * (2.0 * math.pi / self.N_RAYS)
        dx, dy = np.cos(ang), np.sin(ang)

        r_true = np.full(self.N_RAYS, np.inf)
        if len(self.edges):
            ax, ay = self.edges[:, 0] - ox, self.edges[:, 1] - oy
            ex, ey = self.edges[:, 2] - self.edges[:, 0], self.edges[:, 3] - self.edges[:, 1]
            den = dx[:, None] * ey[None] - dy[:, None] * ex[None]
            den = np.where(np.abs(den) < 1e-12, 1e-12, den)
            t = (ax[None] * ey[None] - ay[None] * ex[None]) / den
            s = (ax[None] * dy[:, None] - ay[None] * dx[:, None]) / den
            t = np.where((t > 0.0) & (s >= 0.0) & (s <= 1.0), t, np.inf)
            r_true = t.min(axis=1)
        r_true = np.where(r_true > self.SCAN_RANGE, np.inf, r_true)
        r_meas = r_true + rng.normal(0.0, 0.03 * k, self.N_RAYS)
        r_meas[rng.random(self.N_RAYS) < 0.02 * k] = np.nan

        dets = []
        dth = 2.0 * math.pi / self.N_RAYS
        for pts in self.line_pts:
            rx, ry = pts[:, 0] - ox, pts[:, 1] - oy
            dist = np.hypot(rx, ry)
            idx = np.round((np.arctan2(ry, rx) - pose[2]) / dth).astype(int) % self.N_RAYS
            vis = (dist < self.LINE_RANGE) & (dist < r_true[idx] - 0.15)
            if not vis.any():
                continue
            edge = np.flatnonzero(np.diff(np.concatenate(([0], vis.astype(np.int8), [0]))))
            for i0, i1 in zip(edge[0::2], edge[1::2] - 1):
                if i1 - i0 < 3 or rng.random() < 0.12 * k:
                    continue
                if rng.random() < 0.2 * k and i1 - i0 > 8:       # fragmented detection
                    j0 = int(rng.integers(i0, i1 - 4))
                    i0, i1 = j0, int(rng.integers(j0 + 4, i1 + 1))
                u = pts[-1] - pts[0]
                u = u / np.hypot(*u)
                nv = np.array([-u[1], u[0]])
                bias = rng.normal(0.0, 0.02 * k)
                seg = []
                for i in (i0, i1):
                    r = dist[i]
                    seg.append(pts[i] + nv * (bias + rng.normal(0.0, (0.03 + 0.012 * r) * k))
                               + u * rng.normal(0.0, (0.08 + 0.03 * r) * k))
                dets.append((seg[0][0], seg[0][1], seg[1][0], seg[1][1],
                             0.5 * (dist[i0] + dist[i1])))
        for _ in range(rng.poisson(0.25 * k)):                     # clutter
            r, b = rng.uniform(2.0, self.LINE_RANGE), rng.uniform(0.0, 2.0 * math.pi)
            a, half = rng.uniform(0.0, math.pi), 0.5 * rng.uniform(0.8, 2.5)
            cx, cy = ox + r * math.cos(b), oy + r * math.sin(b)
            dets.append((cx - half * math.cos(a), cy - half * math.sin(a),
                         cx + half * math.cos(a), cy + half * math.sin(a), r))
        # a dropped ray (NaN) says nothing; a ray that hit nothing (inf) is free over the whole range
        r_hit = np.where(np.isfinite(r_meas), r_meas, np.nan)
        r_free = np.where(np.isnan(r_meas), 0.0, np.minimum(r_meas, self.SCAN_RANGE) - 0.1)
        return [((ox, oy), ang, r_hit, r_free)], dets


# =============================================================================
# Perception from Chrono::Sensor. The car carries what a production car could: a stereo pair of
# cameras behind the windshield, one camera at the tail, one on the front bumper, and optionally
# a forward-facing lidar. Nothing looks sideways: what is beside the car is known only from what
# was seen before. Range is not read from the renderer. It is computed from the images, by a
# stereo network for the pair and by a monocular depth network for the single cameras, both of
# which run in stereo_worker.py.
# =============================================================================

def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pinhole_rays(w, h, hfov):
    """Unit ray of every pixel in the camera frame (x forward, y left, z up). Chrono image
    buffers start at the bottom-left pixel."""
    f = 0.5 * w / math.tan(0.5 * hfov)
    U, V = np.meshgrid(np.arange(w) + 0.5 - 0.5 * w, np.arange(h) + 0.5 - 0.5 * h)
    d = np.stack([np.full_like(U, f), -U, V], axis=-1)
    return (d / np.linalg.norm(d, axis=-1, keepdims=True)).astype(np.float32)


def angular_rays(w, h, hfov, el_min, el_max, inclusive):
    """Unit rays of a scanning sensor: one per bearing and elevation. The lidar spaces its beams
    from end to end of each range (inclusive); otherwise they are at the centres of the cells."""
    if inclusive:
        az = -0.5 * hfov + np.arange(w) / (w - 1) * hfov
        el = el_min + np.arange(h) / (h - 1) * (el_max - el_min)
    else:
        az = -0.5 * hfov + (np.arange(w) + 0.5) / w * hfov
        el = el_min + (np.arange(h) + 0.5) / h * (el_max - el_min)
    EL, AZ = np.meshgrid(el, az, indexing="ij")
    return np.stack([np.cos(EL) * np.cos(AZ), np.cos(EL) * np.sin(AZ), np.sin(EL)], axis=-1).astype(np.float32)


def planar_scan(P, origin, obstacle, ground, blocking, n_bins, half_fov, heading, r_max, r_cell=0.1):
    """Collapse classified 3D points into a planar scan around the sensor: per bearing, the range
    of the nearest obstacle (NaN if none), the range known to be free, the range of the first
    thing that is not ground, and the range of the last ground seen in front of that. Anything
    that is not clearly ground ends the free part of a ray, but only a clear obstacle counts as
    a hit."""
    dx, dy = P[..., 0] - origin[0], P[..., 1] - origin[1]
    rho = np.hypot(dx, dy)
    rel = (np.arctan2(dy, dx) - heading + math.pi) % (2.0 * math.pi) - math.pi
    b = np.floor((rel + half_fov) / (2.0 * half_fov) * n_bins).astype(np.int64)
    ok = (b >= 0) & (b < n_bins) & (rho < r_max)
    nr = int(r_max / r_cell) + 1
    ir = np.minimum((rho / r_cell).astype(np.int64), nr - 1)

    def nearest(mask, few):
        """Range of the first point along each bearing that has company: at least `few` points
        within 0.3 m behind it. A stray pixel does not make an obstacle, and the answer is the
        range of the point itself, so the front edge of a low, flat thing like a kerb is not
        pushed back by all the points on top of it."""
        m = ok & mask
        key = b[m] * nr + ir[m]
        hist = np.bincount(key, minlength=n_bins * nr).reshape(n_bins, nr)
        near = np.full(n_bins * nr, np.inf)
        np.minimum.at(near, key, rho[m])
        pad = np.pad(hist, ((0, 0), (0, 2)))
        good = (hist > 0) & (pad[:, :-2] + pad[:, 1:-1] + pad[:, 2:] >= few)
        first = good.argmax(axis=1)
        return np.where(good.any(axis=1), near.reshape(n_bins, nr)[np.arange(n_bins), first], np.nan)

    few = 3 if P.shape[0] * P.shape[1] > 20000 else 1    # an image has pixels to spare, a scanner has not
    r_hit = nearest(obstacle, few)
    r_stop = nearest(blocking, few)
    m = ok & ground
    far = np.zeros(n_bins)
    np.maximum.at(far, b[m], rho[m])
    m[m] = ~(rho[m] > r_stop[b[m]])                    # (ground in front of the first thing that is not)
    before = np.zeros(n_bins)
    np.maximum.at(before, b[m], rho[m])
    ang = heading - half_fov + (np.arange(n_bins) + 0.5) * (2.0 * half_fov / n_bins)
    return ang, r_hit, np.maximum(np.fmin(far, r_stop - 0.15), 0.0), r_stop, before


def paint_segments(pts, origin, cell=0.05, r_max=14.0, max_lines=20, min_len=0.35, min_count=2):
    """Straight stripes among the ground points classified as paint. A Hough vote finds a line,
    a total least squares fit over its cells refines it, gaps split it into segments. A segment
    is built from solid pieces of at least 0.25 m: a patch of sunlit road between two shadows
    is as light as paint, but it is a spot, and a spot does not start, end or extend a line.
    Returns a list of (x1, y1, x2, y2, distance from the sensor)."""
    if len(pts) < 20:
        return []
    ij = np.floor((pts - origin) / cell).astype(np.int64)
    off = int(r_max / cell) + 2
    ij = ij[(np.abs(ij) < off).all(axis=1)]
    key, cnt = np.unique((ij[:, 0] + off) * (2 * off + 1) + (ij[:, 1] + off), return_counts=True)
    key = key[cnt >= min_count]                           # single pixels are speckle
    if len(key) < 12:
        return []
    c = (np.stack([key // (2 * off + 1) - off, key % (2 * off + 1) - off], axis=1) + 0.5) * cell
    th = np.radians(np.arange(0.0, 180.0, 1.0))
    ct, st = np.cos(th), np.sin(th)
    span = 1.5 * r_max
    nr = int(2.0 * span / cell) + 1

    def votes(q):
        r = q[:, :1] * ct[None, :] + q[:, 1:] * st[None, :]
        return (np.arange(len(th))[None, :] * nr + np.round((r + span) / cell).astype(np.int64)).ravel()

    acc = np.bincount(votes(c), minlength=len(th) * nr)
    alive = np.ones(len(c), dtype=bool)
    out = []
    for _ in range(max_lines):
        k = int(acc.argmax())
        if acc[k] < 5:
            break
        it, ir = divmod(k, nr)
        near = alive & (np.abs(c @ np.array([ct[it], st[it]]) - (ir * cell - span)) < 0.12)
        if near.sum() < 6:
            acc[k] = 0
            continue
        m = c[near].mean(axis=0)
        cov = (c[near] - m).T @ (c[near] - m)
        ang = 0.5 * math.atan2(2.0 * cov[0, 1], cov[0, 0] - cov[1, 1])
        d = np.array([math.cos(ang), math.sin(ang)])
        off_n = (c - m) @ np.array([-d[1], d[0]])
        inl = alive & (np.abs(off_n) < 0.10)
        s = np.sort((c[inl] - m) @ d)
        cut = np.flatnonzero(np.diff(s) > 2.5 * cell)
        solid = [s[a:b + 1] for a, b in zip(np.concatenate(([0], cut + 1)), np.concatenate((cut, [len(s) - 1])))
                 if s[b] - s[a] >= 0.25]
        s = np.concatenate(solid) if solid else s[:0]
        cut = np.flatnonzero(np.diff(s) > 0.45)
        for a, b in zip(np.concatenate(([0], cut + 1)), np.concatenate((cut, [len(s) - 1]))) if len(s) else ():
            if s[b] - s[a] >= min_len and (b - a + 1) >= 0.35 * (s[b] - s[a]) / cell:
                p1, p2 = origin + m + s[a] * d, origin + m + s[b] * d
                out.append((p1[0], p1[1], p2[0], p2[1], float(np.hypot(*(m + 0.5 * (s[a] + s[b]) * d)))))
        acc -= np.bincount(votes(c[inl]), minlength=len(acc))
        alive &= ~inl
    return out
