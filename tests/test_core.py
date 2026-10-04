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


if __name__ == "__main__":
    test_reeds_shepp()
    test_mpc_solver()
    test_footprint_and_distance()
    print("all checks passed")
