#!/usr/bin/env python3
"""Checks of the numerical core that do not need a simulation run.

    python tests/test_core.py

Needs a Python with PyChrono (parking_sim.py imports it), takes about ten seconds."""

import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("parking_sim", os.path.join(HERE, "..", "parking_sim.py"))
ps = importlib.util.module_from_spec(spec)
sys.modules["parking_sim"] = ps
spec.loader.exec_module(ps)


def test_reeds_shepp():
    """Every candidate word must end at the goal, and the scalar and array backends must agree."""
    rng = np.random.default_rng(0)
    words, bad = set(), 0
    for _ in range(2000):
        x, y, phi = rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-math.pi, math.pi)
        paths = ps.rs_paths(x, y, phi)
        assert paths, "no Reeds-Shepp path found"
        for word, lens in paths:
            rows = ps.rs_sample((0.0, 0.0, 0.0), word, lens, 1.0, 0.05)
            err = math.hypot(rows[-1, 0] - x, rows[-1, 1] - y) + abs(ps.wrap(rows[-1, 2] - phi))
            bad += err > 1e-6
            words.add(word)
        # no path can be shorter than the straight-line distance
        assert min(sum(abs(l) for l in lens) for _, lens in paths) >= math.hypot(x, y) - 1e-9
    assert len(words) == 18, "expected all 18 word types, saw %d" % len(words)
    assert bad <= 5, "%d candidates did not end at the goal" % bad      # the planner filters these out
    xs, ph = np.linspace(-5, 5, 11), np.linspace(-3, 3, 7)
    table = ps.rs_length_table(xs, xs, ph)
    for i, j, k in ((0, 2, 0), (5, 9, 3), (10, 4, 6), (7, 7, 2)):
        best = min(sum(abs(l) for l in lens) for _, lens in ps.rs_paths(xs[i], xs[j], ph[k]))
        assert abs(best - table[i, j, k]) < 1e-5
    print("Reeds-Shepp: 18 word types, %d of ~13000 candidates off the goal, backends agree" % bad)


def _qp(mpc, d, e0, psi0, k_prev, k_ref):
    """The dense QP the MPC solves, rebuilt independently of LateralMPC.solve."""
    q, N = mpc.qp[d], mpc.N
    w = np.tile([float(mpc.Q_E), float(mpc.Q_PSI)], N)
    w[-2:] *= mpc.Q_END
    c = np.zeros(N)
    c[0] = k_prev
    Hm = 2.0 * ((q["Gam"].T * w) @ q["Gam"] + mpc.R_K * np.eye(N) + mpc.R_DK * mpc.D.T @ mpc.D)
    f = 2.0 * ((q["Gam"].T * w) @ (q["Phi"] @ np.array([e0, psi0]) - q["Gam"] @ k_ref) - mpc.R_K * k_ref
               - mpc.R_DK * mpc.D.T @ c)
    return Hm, f, c


def test_mpc_solver():
    """The MPC's QP solve against the closed form (no active constraints) and against a slow
    projected-gradient solve of the same problem (active constraints)."""
    mpc = ps.LateralMPC()
    N = mpc.N
    rng = np.random.default_rng(1)
    worst_free, worst_viol, worst_gap, iters = 0.0, 0.0, -1.0, []
    for trial in range(400):
        d = 1 if trial % 2 == 0 else -1
        e0, psi0, k_max = rng.normal(0, 0.10), rng.normal(0, 0.06), 0.17
        k_prev = rng.uniform(-k_max, k_max)
        k_ref = np.where(np.arange(N) < rng.integers(0, N), rng.choice([-0.168, 0.0, 0.168]),
                         rng.choice([-0.168, 0.0, 0.168])).astype(float)
        Hm, f, c = _qp(mpc, d, e0, psi0, k_prev, k_ref)
        if trial % 4 == 0:                                   # limits far away: closed form applies
            K, _ = mpc.solve(d, e0, psi0, k_prev, k_ref, 50.0, 50.0)
            worst_free = max(worst_free, np.abs(K - np.linalg.solve(Hm, -f)).max())
            continue
        dk = float(rng.choice([0.005, 0.01, 0.04, 0.3]))
        K, _ = mpc.solve(d, e0, psi0, k_prev, k_ref, k_max, dk)
        iters.append(mpc.iters)
        worst_viol = max(worst_viol, np.abs(K).max() - k_max, (np.abs(mpc.D @ K - c) - dk).max())
        if trial < 80:
            step = 1.0 / np.linalg.eigvalsh(Hm).max()

            def project(v):                                  # onto the box and rate limits, front to back
                out, prev = np.empty(N), k_prev
                for k in range(N):
                    out[k] = prev = min(max(min(max(v[k], prev - dk), prev + dk), -k_max), k_max)
                return out
            R = project(np.zeros(N))
            for _ in range(30000):
                R = project(R - step * (Hm @ R + f))
            cost = lambda v: 0.5 * v @ Hm @ v + f @ v
            worst_gap = max(worst_gap, (cost(K) - cost(R)) / max(abs(cost(R)), 1e-9))
    assert worst_free < 1e-9, worst_free
    assert worst_viol < 1e-9, worst_viol
    assert worst_gap < 1e-6, worst_gap                       # never worse than the reference solve
    print("MPC QP: closed-form error %.1e, constraint violation %.1e, cost never above the reference "
          "(worst gap %.1e), %.0f iterations on average" % (worst_free, max(worst_viol, 0.0), worst_gap, np.mean(iters)))


def test_footprint_and_distance():
    old = (ps.EGO.rear, ps.EGO.front, ps.EGO.half_width)
    ps.EGO.rear, ps.EGO.front, ps.EGO.half_width = 1.0, 3.8, 0.9
    try:
        poses = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, math.pi / 2]])
        pts = np.array([[3.0, 0.0], [0.0, 3.0], [5.0, 5.0]])
        assert ps.footprint_hits(poses, pts[:1], 0.0).tolist() == [True, False]
        assert ps.footprint_hits(poses, pts[1:2], 0.0).tolist() == [False, True]
        assert not ps.footprint_hits(poses, pts[2:], 0.5).any()
        a = ps.rect_poly(0.0, 0.0, 0.0, -1.0, 1.0, 1.0)
        assert ps.poly_distance(a, ps.rect_poly(4.0, 0.0, 0.0, -1.0, 1.0, 1.0)) == 2.0
        assert ps.poly_distance(a, ps.rect_poly(1.5, 0.5, 0.3, -1.0, 1.0, 1.0)) == 0.0
    finally:
        ps.EGO.rear, ps.EGO.front, ps.EGO.half_width = old
    print("footprint test and polygon distance: ok")


def test_sensor_geometry():
    """Pixel rays, the planar scan made from classified 3D points, and the stripe detector."""
    w, h, hfov = 96, 54, math.radians(120.0)
    rays = ps.pinhole_rays(w, h, hfov)
    assert np.allclose(np.linalg.norm(rays, axis=-1), 1.0, atol=1e-6)
    assert rays[0, 0, 1] > 0 and rays[0, 0, 2] < 0, "the buffer starts at the bottom-left pixel"
    edge = math.atan2(rays[h // 2, 0, 1], rays[h // 2, 0, 0])
    assert abs(edge - 0.5 * hfov) < math.radians(0.7), "the first column looks half the field of view to the left"

    # a wall 5 m ahead, ground in front of it, and a kerb (flat top, 0.6 m deep) off to the left
    a = np.radians(np.arange(-40.0, 40.0, 0.25))
    wall = np.stack([np.full_like(a, 5.0), 5.0 * np.tan(a) * 0.2, np.zeros_like(a)], axis=1)
    wall = np.concatenate([wall + (0.0, 0.0, z) for z in (0.3, 0.6, 0.9)])
    gx, gy = np.meshgrid(np.arange(0.5, 9.0, 0.05), np.arange(-4.0, 4.0, 0.05))
    ground = np.stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)], axis=1)
    kx, ky = np.meshgrid(np.arange(1.0, 4.0, 0.05), np.arange(3.0, 3.6, 0.05))
    kerb = np.stack([kx.ravel(), ky.ravel(), np.full(kx.size, 0.15)], axis=1)
    P = np.concatenate([wall, ground, kerb])[None]
    obstacle, on_ground = P[..., 2] > 0.1, np.abs(P[..., 2]) < 0.05
    ang, r_hit, r_free, r_stop, before = ps.planar_scan(P, (0.0, 0.0), obstacle, on_ground, ~on_ground, 240,
                                                        math.radians(60.0), 0.0, 12.0)
    assert np.all(before <= np.where(np.isfinite(r_stop), r_stop, np.inf) + 1e-9)
    ahead = np.abs(ang) < math.radians(8.0)
    assert np.allclose(r_hit[ahead], 5.0 / np.cos(ang[ahead]) * 1.0, atol=0.12), r_hit[ahead]
    assert np.all(r_free[ahead] < r_hit[ahead]) and np.all(r_free[ahead] > r_hit[ahead] - 0.4)
    side = (ang > math.radians(50.0)) & (ang < math.radians(58.0))
    # the kerb is hit at its near edge (y = 3.0), not somewhere on its 0.6 m deep top
    assert np.allclose(r_hit[side] * np.sin(ang[side]), 3.0, atol=0.08), r_hit[side] * np.sin(ang[side])

    # two collinear stripes with a gap, one stripe across them, and speckle
    rng = np.random.default_rng(3)
    def stripe(p, q, n):
        t = rng.random((n, 1))
        d = np.array(q) - np.array(p)
        nrm = np.array([-d[1], d[0]]) / np.hypot(*d)
        return np.array(p) + t * d + rng.uniform(-0.06, 0.06, (n, 1)) * nrm
    pts = np.concatenate([stripe((2.0, 1.0), (2.0, 4.0), 900), stripe((2.0, 5.5), (2.0, 8.0), 800),
                          stripe((4.0, 2.0), (7.0, 2.5), 900), rng.uniform(0.0, 9.0, (60, 2))])
    segs = ps.paint_segments(pts, np.zeros(2))
    found = sorted((round(min(s[1], s[3]), 1), round(max(s[1], s[3]), 1)) for s in segs if abs(s[0] - 2.0) < 0.05
                   and abs(s[2] - 2.0) < 0.05)
    assert found == [(1.0, 4.0), (5.5, 8.0)], found
    assert any(abs(math.hypot(s[2] - s[0], s[3] - s[1]) - math.hypot(3.0, 0.5)) < 0.15 for s in segs)
    assert len(segs) == 3, "speckle must not become a line: %d segments" % len(segs)
    print("pixel rays, planar scan and stripe detector: ok")


def test_mapping():
    """Line tracks that remember where paint was seen, and the tentative layer of the grid."""
    trk = ps.LineTrack((0.0, 0.0, 0.0, 2.0, 8.0), 0.0, keep=True)       # the mouth of a line, seen from far
    for _ in range(12):
        trk.add((0.0, 0.0, 0.0, 2.0, 8.0))
    for _ in range(200):                                                # then only its far end, from close
        trk.add((0.0, 3.0, 0.0, 5.5, 2.0))
    lo = min(e[1] for e in trk.ends())
    assert lo < 0.15, "the track forgot the stretch it saw from far away (starts at %.2f)" % lo
    plain = ps.LineTrack((0.0, 0.0, 0.0, 2.0, 8.0), 0.0)
    for _ in range(12):
        plain.add((0.0, 0.0, 0.0, 2.0, 8.0))
    for _ in range(200):
        plain.add((0.0, 3.0, 0.0, 5.5, 2.0))
    assert min(e[1] for e in plain.ends()) > 2.5                        # (what happens without the memory)

    old = (ps.EGO.rear, ps.EGO.front, ps.EGO.half_width, ps.EGO.length)
    ps.EGO.rear, ps.EGO.front, ps.EGO.half_width, ps.EGO.length = 1.0, 3.8, 0.9, 4.8
    try:
        g = ps.GridMap((0.0, -5.0, 20.0, 5.0))
        ang = np.radians(np.arange(-30.0, 30.0, 0.5))
        none = np.full(len(ang), np.nan)
        for _ in range(4):      # sure of the first 6 m, ground seen up to 11 m
            g.update((0.0, 0.0), ang, none, np.full(len(ang), 6.0), np.full(len(ang), 11.0))
        cell = lambda x, y: (int((y - g.y0) / g.RES), int((x - g.x0) / g.RES))
        b = g.blocked()
        assert not b[cell(4.0, 0.0)] and not b[cell(9.0, 0.0)], "seen ground must be drivable"
        assert b[cell(10.8, 0.0)], "the last 0.4 m of far ground are kept as a margin"
        assert b[cell(14.0, 0.0)] and b[cell(4.0, 4.0)], "what was never seen is not drivable"
        assert g.free[cell(9.0, 0.0)] == 0, "far ground is no evidence that a cell is empty"
        stop = np.where(np.abs(ang) < 0.05, 5.5, np.nan)            # a closer look finds something at x = 9
        g.update((3.5, 0.0), ang, none, np.full(len(ang), 5.0), None, stop)
        assert g.blocked()[cell(9.0, 0.0)], "something seen there overrules the far ground"
        g.mark_free((15.0, 3.0, 0.0))
        assert not g.blocked()[cell(16.0, 3.0)], "the ground under the car is free"
    finally:
        ps.EGO.rear, ps.EGO.front, ps.EGO.half_width, ps.EGO.length = old
    print("line memory and tentative map layer: ok")


def test_pictures():
    """The pictures of the viewer are made at the size they are drawn at."""
    flat = np.full((540, 960, 3), (10, 120, 250), np.uint8)
    out = ps.resample(flat, 400, 225)
    assert out.shape == (225, 400, 3) and out.dtype == np.uint8 and np.all(out == (10, 120, 250))
    board = np.zeros((540, 960, 3), np.uint8)              # one pixel on, one off: averages to grey
    board[::2, ::2] = board[1::2, 1::2] = 255
    out = ps.resample(board, 400, 225)
    assert out.min() >= 126 and out.max() <= 128
    ramp = np.repeat((np.arange(480) // 2).astype(np.uint8)[None, :, None], 3, axis=2).repeat(270, axis=0)
    out = ps.resample(ramp, 400, 225)                      # a gradient stays one, end to end
    assert np.all(np.diff(out[100, :, 0].astype(int)) >= 0) and out[100, 0, 0] <= 1 and out[100, -1, 0] >= 238
    assert np.array_equal(ps.resample(ramp, 480, 270), ramp)
    assert ps.RAMP.shape == (256, 3) and tuple(ps.RAMP[0]) == (46, 58, 150) and tuple(ps.RAMP[-1]) == (222, 44, 32)
    print("picture resampling: ok")


if __name__ == "__main__":
    test_reeds_shepp()
    test_mpc_solver()
    test_footprint_and_distance()
    test_sensor_geometry()
    test_mapping()
    test_pictures()
    print("all checks passed")
