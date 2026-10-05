"""Planning: configuration space, the holonomic heuristic, Hybrid A* and the path it returns."""

import heapq
import math
import time

import numpy as np

from .config import STEER_RATE, V_FWD, V_REV
from .geometry import wrap
from .reeds_shepp import rs_length_table, rs_paths, rs_sample
from .vehicle import EGO


def _fast_len(n):
    while True:
        m = n
        for p in (2, 3, 5):
            while m % p == 0:
                m //= p
        if m == 1:
            return n
        n += 1


class CSpace:
    NTH = 120
    FREE, SOFT, HARD = 0, 1, 2
    SOFT_BAND = 0.30

    def __init__(self, occ, x0, y0, res, m_lat, m_lon):
        self.x0, self.y0, self.res = x0, y0, res
        self.ny, self.nx = occ.shape
        pad = int((EGO.front + m_lon + self.SOFT_BAND) / res) + 4
        fy, fx = _fast_len(self.ny + pad), _fast_len(self.nx + pad)
        O = np.fft.rfft2(occ.astype(np.float64), s=(fy, fx))
        r = pad - 2
        oy, ox = np.mgrid[-r:r + 1, -r:r + 1]
        big = 4096.0
        self.cost = np.zeros((self.NTH, self.ny, self.nx), dtype=np.uint8)
        half = self.NTH // 2
        for k in range(half):
            th = k * 2.0 * math.pi / self.NTH
            c, s = math.cos(th), math.sin(th)
            lx = (ox * c + oy * s) * res
            ly = (-ox * s + oy * c) * res
            e = 0.5 * res
            hard = ((lx > -EGO.rear - m_lon - e) & (lx < EGO.front + m_lon + e) &
                    (np.abs(ly) < EGO.half_width + m_lat + e))
            b = self.SOFT_BAND
            soft = ((lx > -EGO.rear - m_lon - b) & (lx < EGO.front + m_lon + b) &
                    (np.abs(ly) < EGO.half_width + m_lat + b))
            K = np.zeros((fy, fx))
            K[oy[soft] % fy, ox[soft] % fx] = 1.0
            K[oy[hard] % fy, ox[hard] % fx] = big
            F = np.fft.rfft2(K)
            # the footprint at heading + pi is the point reflection of this one
            for kk, G in ((k, np.conj(F)), (k + half, F)):
                C = np.fft.irfft2(O * G, s=(fy, fx))[:self.ny, :self.nx]
                self.cost[kk] = np.where(C > big - 0.5, self.HARD, np.where(C > 0.5, self.SOFT, self.FREE))
        self._kth = self.NTH / (2.0 * math.pi)

    def query(self, x, y, th):
        fx, fy = (x - self.x0) / self.res, (y - self.y0) / self.res
        if fx < 0.0 or fy < 0.0 or fx >= self.nx or fy >= self.ny:
            return self.HARD
        return self.cost[int(round(th * self._kth)) % self.NTH, int(fy), int(fx)]

    def query_many(self, x, y, th):
        fx, fy = (x - self.x0) / self.res, (y - self.y0) / self.res
        ok = (fx >= 0.0) & (fy >= 0.0) & (fx < self.nx) & (fy < self.ny)
        out = np.full(len(x), self.HARD, dtype=np.uint8)
        k = np.round(th[ok] * self._kth).astype(int) % self.NTH
        out[ok] = self.cost[k, fy[ok].astype(int), fx[ok].astype(int)]
        return out


def holonomic_distance(occ, res, goal_xy, x0, y0, radius=0.8, block=4):
    """Obstacle-aware 2D distance-to-goal on a coarse grid (Dijkstra), for the heuristic."""
    ny, nx = occ.shape
    r = int(radius / res)
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    disc = (xx * xx + yy * yy) <= r * r
    fy, fx = _fast_len(ny + r + 2), _fast_len(nx + r + 2)
    K = np.zeros((fy, fx))
    K[yy[disc] % fy, xx[disc] % fx] = 1.0
    fat = np.fft.irfft2(np.fft.rfft2(occ.astype(np.float64), s=(fy, fx)) * np.fft.rfft2(K), s=(fy, fx))
    fat = fat[:ny, :nx] > 0.5
    my, mx = ny // block, nx // block
    blocked = fat[:my * block, :mx * block].reshape(my, block, mx, block).all(axis=(1, 3))
    cell = res * block
    gi, gj = int((goal_xy[0] - x0) / cell), int((goal_xy[1] - y0) / cell)
    gi, gj = min(max(gi, 0), mx - 1), min(max(gj, 0), my - 1)
    blocked[max(gj - 1, 0):gj + 2, max(gi - 1, 0):gi + 2] = False
    dist = np.full((my, mx), np.inf)
    dist[gj, gi] = 0.0
    heap = [(0.0, gi, gj)]
    nbrs = [(-1, -1, 1.4142), (0, -1, 1.0), (1, -1, 1.4142), (-1, 0, 1.0),
            (1, 0, 1.0), (-1, 1, 1.4142), (0, 1, 1.0), (1, 1, 1.4142)]
    while heap:
        d, i, j = heapq.heappop(heap)
        if d > dist[j, i]:
            continue
        for di, dj, w in nbrs:
            a, b = i + di, j + dj
            if 0 <= a < mx and 0 <= b < my and not blocked[b, a]:
                nd = d + w * cell
                if nd < dist[b, a]:
                    dist[b, a] = nd
                    heapq.heappush(heap, (nd, a, b))
    return dist, cell


# =============================================================================
# Hybrid A* over (x, y, heading) with forward and reverse motion primitives and
# Reeds-Shepp analytic expansions to the goal
# =============================================================================

class Planner:
    XY_RES = 0.35
    NTH = 60
    PRIM = 0.7                  # arc length of one motion primitive [m]
    W_REV = 1.5                 # reversing costs more per metre than driving forward
    W_SWITCH = 5.0              # cost of a direction change [m]
    W_DSTEER = 0.4              # cost of a full-lock steering change [m]
    W_SOFT = 1.5                # extra cost per metre driven close to an obstacle
    TAB_RANGE = 22.0
    TAB_RES = 0.5
    TAB_NPHI = 72

    def __init__(self):
        self._tab = None
        self.max_iter = 30000       # expansion limit for one search (keeps runs repeatable)
        self.prims = []
        for d in (1, -1):
            kmax = EGO.kappa
            for k in (-kmax, -0.5 * kmax, 0.0, 0.5 * kmax, kmax):
                subs = []
                for frac in (0.5, 1.0):
                    s = d * self.PRIM * frac
                    if k == 0.0:
                        subs.append((s, 0.0, 0.0))
                    else:
                        subs.append((math.sin(k * s) / k, (1.0 - math.cos(k * s)) / k, k * s))
                self.prims.append((k, d, subs, self.PRIM * (1.0 if d > 0 else self.W_REV)))

    def table(self):
        if self._tab is None:
            n = int(round(self.TAB_RANGE / self.TAB_RES))
            ax = np.arange(-n, n + 1) * self.TAB_RES / EGO.radius
            ph = np.arange(self.TAB_NPHI) * 2.0 * math.pi / self.TAB_NPHI
            ph = np.where(ph > math.pi, ph - 2.0 * math.pi, ph)
            self._tab = (rs_length_table(ax, ax, ph) * EGO.radius).astype(np.float32)
        return self._tab

    def _rs_shot(self, pose, d0, goal, cs):
        """Cheapest collision-free Reeds-Shepp connection from pose to goal, or None."""
        x, y, th = pose
        dx, dy = goal[0] - x, goal[1] - y
        c, s = math.cos(th), math.sin(th)
        cands = []
        for word, lens in rs_paths((dx * c + dy * s) / EGO.radius, (-dx * s + dy * c) / EGO.radius,
                                   wrap(goal[2] - th)):
            cost, prev = 0.0, d0
            for l in lens:
                if abs(l) * EGO.radius < 1e-4:
                    continue
                d = 1 if l > 0 else -1
                cost += abs(l) * EGO.radius * (1.0 if d > 0 else self.W_REV)
                if prev != 0 and d != prev:
                    cost += self.W_SWITCH
                prev = d
            cands.append((cost, word, lens, prev))
        cands.sort(key=lambda q: q[0])
        for cost, word, lens, last in cands[:6]:
            rows = rs_sample(pose, word, lens, EGO.radius, 0.1)
            if len(rows) == 0:
                return 0.0, rows, d0
            if (math.hypot(rows[-1, 0] - goal[0], rows[-1, 1] - goal[1]) > 0.03 or
                    abs(wrap(rows[-1, 2] - goal[2])) > 0.01):
                continue
            chk = np.concatenate([rows[::2], rows[-1:]])
            v = cs.query_many(chk[:, 0], chk[:, 1], chk[:, 2])
            if (v == CSpace.HARD).any():
                continue
            return cost + self.W_SOFT * 0.2 * float((v == CSpace.SOFT).sum()), rows, last
        return None

    def _arc_shot(self, pose, d0, dock, cs):
        """Straight, one arc onto the stall axis, straight to the goal, all in the docking
        direction. Tried with the tightest arc that needs no lead-in and a few fixed radii."""
        (gx, gy, gth), sg = dock["goal"], dock["sign"]
        x, y, th = pose
        c, s = math.cos(gth), math.sin(gth)
        lx, ly = (x - gx) * c + (y - gy) * s, -(x - gx) * s + (y - gy) * c
        lth = wrap(th - gth)
        if abs(lth) > 1.75:
            return None
        kmax = EGO.kappa
        if abs(lth) < 0.004:
            options = [(0.0, 0.0)] if abs(ly) < 0.02 else []
        else:
            options = []
            if abs(ly) > 1e-3:
                options.append(((1.0 - math.cos(lth)) / ly, 0.0))
            for frac in (1.0, 0.7, 0.45):
                k = -sg * math.copysign(frac * kmax, lth)
                options.append((k, ((1.0 - math.cos(lth)) / k - ly) / (sg * math.sin(lth))))
        best = None
        wdir = 1.0 if sg > 0 else self.W_REV
        for k, lead in options:
            if abs(k) > kmax + 1e-9 or lead < 0.0 or lead > 25.0:
                continue
            s_arc = -lth / k if k != 0.0 else 0.0
            if s_arc * sg < 0.0 or abs(s_arc) > 30.0:
                continue
            x1 = lx + lead * sg * math.cos(lth) - (math.sin(lth) / k if k != 0.0 else 0.0)
            run = -sg * x1                  # straight left to drive along the stall axis
            if run < 1.0 or run > 40.0:
                continue
            cost = (lead + abs(s_arc) + run) * wdir + self.W_DSTEER * abs(k) / EGO.kappa
            if best is not None and cost >= best[0]:
                continue
            rows = []
            px, py = x, y
            if lead > 1e-3:
                n = int(math.ceil(lead / 0.1))
                f = np.linspace(lead / n, lead, n) * sg
                rows.append(np.stack([x + f * math.cos(th), y + f * math.sin(th), np.full(n, th),
                                      np.full(n, float(sg)), np.zeros(n)], axis=1))
                px, py = rows[-1][-1, 0], rows[-1][-1, 1]
            if k != 0.0:
                n = max(1, int(math.ceil(abs(s_arc) / 0.1)))
                sa = np.linspace(s_arc / n, s_arc, n)
                pth = th + k * sa
                rows.append(np.stack([px + (np.sin(pth) - math.sin(th)) / k, py - (np.cos(pth) - math.cos(th)) / k,
                                      pth, np.full(n, float(sg)), np.full(n, k)], axis=1))
            n = int(math.ceil(run / 0.1))
            left = run * (1.0 - np.linspace(1.0 / n, 1.0, n))
            rows.append(np.stack([gx - sg * left * c, gy - sg * left * s, np.full(n, gth),
                                  np.full(n, float(sg)), np.zeros(n)], axis=1))
            chk = np.concatenate(rows[:-1] + [rows[-1][left > dock["length"]]])
            v = cs.query_many(chk[:, 0], chk[:, 1], chk[:, 2]) if len(chk) else np.zeros(0, dtype=np.uint8)
            if (v == CSpace.HARD).any():
                continue
            cost += self.W_SOFT * 0.1 * float((v == CSpace.SOFT).sum())
            if best is None or cost < best[0]:
                best = (cost, np.concatenate(rows))
        if best is not None and d0 != 0 and d0 != sg:
            best = (best[0] + self.W_SWITCH, best[1])
        return best

    def shoot(self, pose, d0, goal, cs, dock):
        """Best analytic connection from a node to the final pose (rows end at the parked pose)."""
        best = self._arc_shot(pose, d0, dock, cs) if dock is not None else None
        rs = self._rs_shot(pose, d0, goal, cs)
        if rs is not None:
            cost, rows, last = rs
            if dock is not None:
                cost += dock["cost"] + (self.W_SWITCH if last not in (0, dock["sign"]) else 0.0)
                rows = np.concatenate([rows, dock["rows"]])
            if best is None or cost < best[0]:
                best = (cost, rows)
        return best

    def search(self, start, goal, cs, h2d, h2d_cell, dock=None):
        """Returns rows (x, y, heading, direction, curvature) from start to the parked pose, or
        None. goal is where the search aims; with dock, a straight run from there into the stall
        (dict: goal, sign, length) is appended to whatever connection reaches it."""
        if dock is not None:
            n = int(math.ceil(dock["length"] / 0.1))
            f = np.linspace(1.0 / n, 1.0, n)[:, None]
            xy = np.array(goal[:2]) + f * (np.array(dock["goal"][:2]) - np.array(goal[:2]))
            dock = dict(dock, cost=dock["length"] * (1.0 if dock["sign"] > 0 else self.W_REV),
                        rows=np.concatenate([xy, np.full((n, 1), goal[2]), np.full((n, 1), float(dock["sign"])),
                                             np.zeros((n, 1))], axis=1))
        tail = dock["cost"] if dock is not None else 0.0
        tab = self.table()
        n_tab = int(round(self.TAB_RANGE / self.TAB_RES))
        k_phi = self.TAB_NPHI / (2.0 * math.pi)
        gx, gy, gth = goal
        x0, y0, inv, nx, ny = cs.x0, cs.y0, 1.0 / cs.res, cs.nx, cs.ny
        cost_arr, kth, nthc = cs.cost, cs._kth, cs.NTH
        inv_xy, k_key = 1.0 / self.XY_RES, self.NTH / (2.0 * math.pi)
        hny, hnx = h2d.shape
        inv_h = 1.0 / h2d_cell

        def heur(x, y, th):
            dx, dy = gx - x, gy - y
            c, s = math.cos(th), math.sin(th)
            i = int(round((dx * c + dy * s) / self.TAB_RES)) + n_tab
            j = int(round((-dx * s + dy * c) / self.TAB_RES)) + n_tab
            if 0 <= i <= 2 * n_tab and 0 <= j <= 2 * n_tab:
                h = float(tab[i, j, int(round(wrap(gth - th) * k_phi)) % self.TAB_NPHI])
            else:
                h = math.hypot(dx, dy)
            a, b = int((x - x0) * inv_h), int((y - y0) * inv_h)
            if 0 <= a < hnx and 0 <= b < hny:
                h2 = h2d[b, a]
                if h2 < 1e9 and h2 > h:
                    h = float(h2)
            return h + tail

        def key(x, y, th):
            return (int((x - x0) * inv_xy), int((y - y0) * inv_xy), int(round(th * k_key)) % self.NTH)

        sk = key(*start)
        nodes = {sk: (0.0, start[0], start[1], start[2], 0, 0.0, None)}
        heap = [(heur(*start), 0, sk, 0.0)]
        closed = set()
        best, found_at, it, tie = None, 0, 0, 1
        t0 = time.time()
        while heap:
            f, _, k, g = heapq.heappop(heap)
            if k in closed:
                continue
            if best is not None and (f >= best[0] or it - found_at > 1000):
                break
            closed.add(k)
            it += 1
            if it > self.max_iter:
                break
            g, x, y, th, d0, k0, _ = nodes[k]

            hr = heur(x, y, th) - tail
            if it == 1 or it % (1 if hr < 6.0 else (3 if hr < 14.0 else 8)) == 0:
                sol = self.shoot((x, y, th), d0, goal, cs, dock)
                if sol is not None and (best is None or g + sol[0] < best[0]):
                    if best is None:
                        found_at = it
                    best = (g + sol[0], k, sol[1])

            c, s = math.cos(th), math.sin(th)
            for pk, pd, subs, base in self.prims:
                soft = 0
                for dx, dy, dth in subs:
                    X, Y, TH = x + dx * c - dy * s, y + dx * s + dy * c, th + dth
                    fx, fy = (X - x0) * inv, (Y - y0) * inv
                    if fx < 0.0 or fy < 0.0 or fx >= nx or fy >= ny:
                        soft = 2
                        break
                    v = cost_arr[int(round(TH * kth)) % nthc, int(fy), int(fx)]
                    if v == 2:
                        soft = 2
                        break
                    soft |= v
                if soft == 2:
                    continue
                nk = key(X, Y, TH)
                if nk in closed:
                    continue
                ng = g + base + self.W_DSTEER * abs(pk - k0) / EGO.kappa
                if d0 != 0 and pd != d0:
                    ng += self.W_SWITCH
                if soft:
                    ng += self.W_SOFT * self.PRIM
                old = nodes.get(nk)
                if old is not None and old[0] <= ng:
                    continue
                nodes[nk] = (ng, X, Y, TH, pd, pk, k)
                heapq.heappush(heap, (ng + 1.1 * heur(X, Y, TH), tie, nk, ng))
                tie += 1

        self.stats = dict(iterations=it, nodes=len(nodes), time=time.time() - t0,
                          cost=best[0] if best is not None else float("inf"))
        self.explored = np.array([(n[1], n[2]) for n in nodes.values()])    # for display only
        if best is None:
            return None
        chain = []
        k = best[1]
        while k is not None:
            chain.append(nodes[k])
            k = chain[-1][6]
        chain.reverse()
        rows = []
        for parent, node in zip(chain[:-1], chain[1:]):
            _, x, y, th, _, _, _ = parent
            d, kap = node[4], node[5]
            s = np.linspace(0.1, self.PRIM, 7) * d
            if kap == 0.0:
                px, py, pth = x + s * math.cos(th), y + s * math.sin(th), np.full(7, th)
            else:
                pth = th + kap * s
                px = x + (np.sin(pth) - math.sin(th)) / kap
                py = y - (np.cos(pth) - math.cos(th)) / kap
            rows.append(np.stack([px, py, pth, np.full(7, float(d)), np.full(7, kap)], axis=1))
        rows.append(best[2])
        return np.concatenate(rows)


class Segment:
    """A stretch of path driven in one direction (+1 forward, -1 reverse)."""

    def __init__(self, pts, kappa, direction, v_max):
        pts = np.asarray(pts, dtype=float)
        self.x, self.y, self.th = pts[:, 0].copy(), pts[:, 1].copy(), pts[:, 2].copy()
        self.dir = int(direction)
        self.v_max = v_max
        self.s = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(self.x), np.diff(self.y)))])
        self.length = float(self.s[-1])
        self.kappa = np.asarray(kappa, dtype=float)
        # Speed limit from the steering actuator: where the path curvature changes by dk/ds, the
        # steering must move at v * dk/ds, which may not exceed what STEER_RATE allows.
        n = max(1, min(5, (len(self.kappa) - 1) // 2))     # ~1 m moving average
        pad = np.concatenate([np.full(n, self.kappa[0]), self.kappa, np.full(n, self.kappa[-1])])
        smooth = np.convolve(pad, np.ones(2 * n + 1) / (2 * n + 1), mode="valid")
        dk = np.abs(np.gradient(smooth, np.maximum(self.s, 1e-9))) if len(smooth) > 2 else np.zeros(len(smooth))
        self.v_ref = np.minimum(v_max, np.clip(STEER_RATE / EGO.wheelbase / np.maximum(dk, 1e-6), 0.5, v_max))

    def reanchor(self, old, new, beyond=0.0, full=4.0, fade=9.0):
        """Move the path by the rigid transform that takes pose 'old' to pose 'new'. Points
        with less than 'full' metres left to drive (this segment plus 'beyond') move all the
        way, the effect fades out towards 'fade' metres, and the rest stays as planned."""
        w = np.clip((fade - (beyond + self.length - self.s)) / (fade - full), 0.0, 1.0)
        dth = wrap(new[2] - old[2])
        c, s = math.cos(dth), math.sin(dth)
        dx, dy = self.x - old[0], self.y - old[1]
        self.x = self.x + w * (new[0] + dx * c - dy * s - self.x)
        self.y = self.y + w * (new[1] + dx * s + dy * c - self.y)
        self.th = self.th + w * dth

    def poses(self, i0=0, stride=3):
        idx = np.unique(np.concatenate([np.arange(i0, len(self.x), stride), [len(self.x) - 1]]))
        return np.stack([self.x[idx], self.y[idx], self.th[idx]], axis=1)


def split_segments(start, rows):
    """Cut planner output into single-direction segments at the cusps."""
    raw = []
    pts, kap, cur = [tuple(start)], [rows[0, 4]], rows[0, 3]
    for r in rows:
        if r[3] != cur:
            raw.append((pts, kap, cur))
            pts, kap, cur = [pts[-1]], [r[4]], r[3]
        pts.append((r[0], r[1], r[2]))
        kap.append(r[4])
    raw.append((pts, kap, cur))
    merged = []
    for pts, kap, d in raw:
        if math.hypot(pts[-1][0] - pts[0][0], pts[-1][1] - pts[0][1]) < 0.08 and len(pts) < 4:
            continue                                   # too short to be worth a gear change
        if merged and merged[-1][2] == d:
            merged[-1] = (merged[-1][0] + pts[1:], merged[-1][1] + kap[1:], d)
        else:
            merged.append((pts, kap, d))
    return [Segment(p, k, d, V_FWD if d > 0 else V_REV) for p, k, d in merged]
