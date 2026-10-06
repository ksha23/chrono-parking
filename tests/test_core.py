#!/usr/bin/env python3
"""Checks of the numerical core that do not need a simulation run.

    python tests/test_core.py

Needs a Python with PyChrono (parking_sim.py imports it), takes about ten seconds."""

import math
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from parking.control import LateralMPC
from parking.draw import RAMP, resample
from parking.geometry import footprint_hits, poly_distance, rect_poly, wrap
from parking.ground import Ground
from parking.localization import Localization
from parking.mapping import GridMap, LineMap, LineTrack
from parking.paint import STATES, lay, paint_textures
from parking.perception import _rot_y, paint_segments, pinhole_rays, planar_scan
from parking.reeds_shepp import rs_length_table, rs_paths, rs_sample
from parking.scene_net import MARKING
from parking.sensors import SensorRig
from parking.vehicle import EGO
from parking.world import surface_textures
from parking.stalls import JoinedLine, find_slots, join_collinear
from parking.stereo_worker import pack, unpack


def test_reeds_shepp():
    """Every candidate word must end at the goal, and the scalar and array backends must agree."""
    rng = np.random.default_rng(0)
    words, bad = set(), 0
    for _ in range(2000):
        x, y, phi = rng.uniform(-6, 6), rng.uniform(-6, 6), rng.uniform(-math.pi, math.pi)
        paths = rs_paths(x, y, phi)
        assert paths, "no Reeds-Shepp path found"
        for word, lens in paths:
            rows = rs_sample((0.0, 0.0, 0.0), word, lens, 1.0, 0.05)
            err = math.hypot(rows[-1, 0] - x, rows[-1, 1] - y) + abs(wrap(rows[-1, 2] - phi))
            bad += err > 1e-6
            words.add(word)
        # no path can be shorter than the straight-line distance
        assert min(sum(abs(l) for l in lens) for _, lens in paths) >= math.hypot(x, y) - 1e-9
    assert len(words) == 18, "expected all 18 word types, saw %d" % len(words)
    assert bad <= 5, "%d candidates did not end at the goal" % bad      # the planner filters these out
    xs, ph = np.linspace(-5, 5, 11), np.linspace(-3, 3, 7)
    table = rs_length_table(xs, xs, ph)
    for i, j, k in ((0, 2, 0), (5, 9, 3), (10, 4, 6), (7, 7, 2)):
        best = min(sum(abs(l) for l in lens) for _, lens in rs_paths(xs[i], xs[j], ph[k]))
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
    mpc = LateralMPC()
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
    old = (EGO.rear, EGO.front, EGO.half_width)
    EGO.rear, EGO.front, EGO.half_width = 1.0, 3.8, 0.9
    try:
        poses = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, math.pi / 2]])
        pts = np.array([[3.0, 0.0], [0.0, 3.0], [5.0, 5.0]])
        assert footprint_hits(poses, pts[:1], 0.0).tolist() == [True, False]
        assert footprint_hits(poses, pts[1:2], 0.0).tolist() == [False, True]
        assert not footprint_hits(poses, pts[2:], 0.5).any()
        a = rect_poly(0.0, 0.0, 0.0, -1.0, 1.0, 1.0)
        assert poly_distance(a, rect_poly(4.0, 0.0, 0.0, -1.0, 1.0, 1.0)) == 2.0
        assert poly_distance(a, rect_poly(1.5, 0.5, 0.3, -1.0, 1.0, 1.0)) == 0.0
    finally:
        EGO.rear, EGO.front, EGO.half_width = old
    print("footprint test and polygon distance: ok")


def test_sensor_geometry():
    """Pixel rays, the planar scan made from classified 3D points, and the stripe detector."""
    w, h, hfov = 96, 54, math.radians(120.0)
    rays = pinhole_rays(w, h, hfov)
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
    ang, r_hit, r_free, r_stop, before = planar_scan(P, (0.0, 0.0), obstacle, on_ground, ~on_ground, 240,
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
    segs = paint_segments(pts, np.zeros(2))
    found = sorted((round(min(s[1], s[3]), 1), round(max(s[1], s[3]), 1)) for s in segs if abs(s[0] - 2.0) < 0.05
                   and abs(s[2] - 2.0) < 0.05)
    assert found == [(1.0, 4.0), (5.5, 8.0)], found
    assert any(abs(math.hypot(s[2] - s[0], s[3] - s[1]) - math.hypot(3.0, 0.5)) < 0.15 for s in segs)
    assert len(segs) == 3, "speckle must not become a line: %d segments" % len(segs)
    print("pixel rays, planar scan and stripe detector: ok")


def test_mapping():
    """Line tracks that remember where paint was seen, and the tentative layer of the grid."""
    far, near = (0.0, 0.0, 0.0, 2.0, 8.0, 0.4), (0.0, 3.0, 0.0, 5.5, 2.0, 0.4)      # (x, y, x, y, range, frame time)
    trk = LineTrack(far, 0.0, keep=True)                              # the mouth of a line, seen from far
    for _ in range(12):
        trk.add(far, 0.0)
    for _ in range(200):                                                # then only its far end, from close
        trk.add(near, 0.0)
    lo = min(e[1] for e in trk.ends())
    assert lo < 0.15, "the track forgot the stretch it saw from far away (starts at %.2f)" % lo
    plain = LineTrack(far, 0.0)
    for _ in range(12):
        plain.add(far, 0.0)
    for _ in range(200):
        plain.add(near, 0.0)
    assert min(e[1] for e in plain.ends()) > 2.5                        # (what happens without the memory)

    old = (EGO.rear, EGO.front, EGO.half_width, EGO.length)
    EGO.rear, EGO.front, EGO.half_width, EGO.length = 1.0, 3.8, 0.9, 4.8
    try:
        g = GridMap((0.0, -5.0, 20.0, 5.0))
        ang = np.radians(np.arange(-30.0, 30.0, 0.5))
        none = np.full(len(ang), np.nan)
        for _ in range(4):      # sure of the first 6 m, ground seen up to 11 m
            g.update((0.0, 0.0), ang, none, np.full(len(ang), 6.0), np.full(len(ang), 11.0), dt=0.4)
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
        EGO.rear, EGO.front, EGO.half_width, EGO.length = old
    print("line memory and tentative map layer: ok")


def test_pictures():
    """The pictures of the viewer are made at the size they are drawn at."""
    flat = np.full((540, 960, 3), (10, 120, 250), np.uint8)
    out = resample(flat, 400, 225)
    assert out.shape == (225, 400, 3) and out.dtype == np.uint8 and np.all(out == (10, 120, 250))
    board = np.zeros((540, 960, 3), np.uint8)              # one pixel on, one off: averages to grey
    board[::2, ::2] = board[1::2, 1::2] = 255
    out = resample(board, 400, 225)
    assert out.min() >= 126 and out.max() <= 128
    ramp = np.repeat((np.arange(480) // 2).astype(np.uint8)[None, :, None], 3, axis=2).repeat(270, axis=0)
    out = resample(ramp, 400, 225)                      # a gradient stays one, end to end
    assert np.all(np.diff(out[100, :, 0].astype(int)) >= 0) and out[100, 0, 0] <= 1 and out[100, -1, 0] >= 238
    assert np.array_equal(resample(ramp, 480, 270), ramp)
    assert RAMP.shape == (256, 3) and tuple(RAMP[0]) == (46, 58, 150) and tuple(RAMP[-1]) == (222, 44, 32)
    # what travels to a depth worker on another machine arrives bit for bit
    rng = np.random.default_rng(3)
    img = rng.integers(0, 256, (60, 96, 3), dtype=np.uint8)
    img[:30] = (np.linspace(60.0, 120.0, 96)[None, :, None] + rng.integers(-2, 3, (30, 96, 3))).astype(np.uint8)    # like a road
    assert np.array_equal(unpack(pack(img[::-1]), img.shape, np.uint8), img[::-1])
    depth = (rng.random((60, 96)) * 300.0).astype(np.float16)
    depth[5, 7], depth[9, 0] = np.inf, 0.0
    back = unpack(pack(depth), depth.shape, np.float16)
    assert back.dtype == np.float16 and np.array_equal(back, depth)
    assert len(pack(img[:30])) < 0.6 * img[:30].nbytes
    print("picture resampling and packing: ok")


def _rig():
    """A sensor rig without sensors: enough of one to run its image processing on made-up data."""
    rig = SensorRig.__new__(SensorRig)
    rig.rays = pinhole_rays(rig.CAM_W // 2, rig.CAM_H // 2, rig.CAM_HFOV)
    rig.linear = ((np.arange(256) / 255.0) ** 2.2).astype(np.float32)
    return rig


def _ground_view(rig, height, pitch, shade=lambda x, y: np.ones_like(x), paint=lambda x, y: np.zeros(x.shape, bool)):
    """What a camera 'height' above the ground and pitched down sees of a flat road: the image
    (bottom row first) and the true depth along the optical axis at half size."""
    R = _rot_y(pitch)
    out = []
    for w, h in ((rig.CAM_W, rig.CAM_H), (rig.CAM_W // 2, rig.CAM_H // 2)):
        d = pinhole_rays(w, h, rig.CAM_HFOV) @ R.T.astype(np.float32)
        t = np.where(d[..., 2] < -1e-3, -height / np.minimum(d[..., 2], -1e-3), np.inf)      # range to the ground
        out.append((d, t))
    (d, t), (dh, th) = out
    x, y = t * d[..., 0], t * d[..., 1]
    lin = np.where(np.isfinite(t), np.where(paint(x, y), 0.60, 0.16) * shade(x, y), 0.5)
    image = np.repeat((255.0 * lin ** (1.0 / 2.2) + 0.5).astype(np.uint8)[..., None], 3, axis=2)
    return image, np.where(np.isfinite(th), th * rig.rays[..., 0], np.inf), R


def test_depth_from_images():
    """Stereo disparity and monocular inverse depth are turned into the right ranges."""
    rig = _rig()
    h = 1.42
    _, z, R = _ground_view(rig, h, 0.0)
    zf = np.repeat(np.repeat(np.minimum(z, 200.0), 2, axis=0), 2, axis=1)                  # full size
    r = rig._stereo_range((rig.CAM_F * rig.BASELINE / zf).astype(np.float32))
    P = r[..., None] * (rig.rays @ R.T.astype(np.float32))
    near = (r > 0.0) & (r < 12.0)
    assert near.sum() > 20000 and np.abs(P[near][:, 2] + h).max() < 2e-3                  # the road comes out flat
    assert np.all(r[:, 4:][z[:, 4:] > rig.CAM_FAR] >= 0.99 * rig.CAM_FAR)                 # and the sky far away
    assert np.all(r[rig.CAM_H // 4 - 40, :2] == 0.0)          # near ground at the left rim: not in the right image
    disp = (rig.CAM_F * rig.BASELINE / zf).astype(np.float32)[::-1]                       # top row first, as it comes back
    rig.rows = (0, rig.CAM_H)
    whole = rig._stereo_rows(disp)
    assert np.array_equal(whole, r)
    rig.rows = (160, 544)                                 # only these rows go to the network: no sky, no bonnet
    part = rig._stereo_rows(disp[160:544])
    lo, hi = (rig.CAM_H - 544) // 2, (rig.CAM_H - 160) // 2                               # the same rows, from the bottom, at half size
    assert np.array_equal(part[lo + 1:hi - 1], whole[lo + 1:hi - 1]) and not part[:lo].any() and not part[hi:].any()
    # the car's lean and height from the road in a depth image: a camera 1.3 m up, the car nose-down by 0.8 degrees
    for pitch_deg, up in ((0.8, 1.30), (-0.5, 1.36), (0.0, 1.33)):
        rig.own, rig.plane, rig.lean, rig.height = (-2.4, 2.4, 0.92), None, np.eye(3), None
        cam = dict(pos=np.array([0.6, 0.15, 1.05]), R=np.eye(3))
        _, zc, Rc = _ground_view(rig, up, math.radians(pitch_deg))
        rng_c = np.where(zc < 60.0, zc / rig.rays[..., 0], 0.0).astype(np.float32)        # range along each ray
        for _ in range(2):
            rig._road_plane(cam, rng_c)
        assert rig.height is not None, "no road found"
        level = rig.lean @ cam["pos"]
        got = math.degrees(math.asin(rig.lean[2, 0]))
        assert abs(level[2] + rig.height - up) < 0.004, (level[2] + rig.height, up)       # the camera is where it is above the road
        assert abs(abs(got) - abs(pitch_deg)) < 0.03 and abs(rig.lean[2, 1]) < 5e-4, (got, pitch_deg)
    step = (rig.CAM_F * rig.BASELINE / zf).astype(np.float32)
    step[:, 480:] += 6.0                                                                  # something nearer on one side
    r = rig._stereo_range(step)               # (rows 100 to 145 show the road from 5 m on, where 6 pixels is a jump)
    assert np.all(r[100:145, 239:241] == 0.0) and np.all(r[100:145, 236] > 0.0) and np.all(r[100:145, 243] > 0.0)

    # a network's inverse depth with an unknown scale and offset, of a road with a wall across it
    cam = dict(pos=np.array([0.0, 0.0, 1.09]), R=_rot_y(math.radians(25.0)))
    _, z, _ = _ground_view(rig, 1.09, math.radians(25.0))
    wall = np.minimum(z, 2.5 / np.maximum(rig.rays[..., 0], 1e-3) * rig.rays[..., 0])       # depth 2.5 m at most
    zf = np.repeat(np.repeat(np.minimum(wall, 30.0), 2, axis=0), 2, axis=1)
    r = rig._mono_range(cam, (3.7 / zf + 0.9).astype(np.float32), np.zeros(3), np.eye(3))
    want = np.minimum(wall, 30.0) / rig.rays[..., 0]
    assert r is not None and np.all(np.abs(r - want)[want < 20.0] < 0.01 * want[want < 20.0] + 0.005)
    assert rig._mono_range(cam, np.full((rig.CAM_H, rig.CAM_W), 0.3, np.float32), np.zeros(3), np.eye(3)) is None
    print("ranges from stereo disparity and from monocular depth: ok")


def test_paint_in_light_and_shade():
    """A stripe is found in the sun and in the shade; the edge of a shadow is not a stripe."""
    rig = _rig()
    h, pitch = 1.09, math.radians(25.0)
    stripe = lambda x, y: (np.abs(y - 0.8) < 0.06) & (x > 1.0) & (x < 4.5)
    shadow = lambda x, y: np.where(x > 2.6, 0.2, 1.0)                                    # a shadow across the road
    image, z, R = _ground_view(rig, h, pitch, shadow, stripe)
    flat = np.isfinite(z)
    xy, _, _ = rig._paint(image, flat, np.array([0.0, 0.0, h]), R.astype(np.float32), 0.0, 5.0)
    assert len(xy) > 80 and np.abs(xy[:, 1] - 0.8).max() < 0.10                           # nothing off the stripe
    along = np.sort(xy[:, 0])
    assert along[0] < 1.15 and along[-1] > 4.3 and np.diff(along).max() < 0.12            # and all of it, lit and shaded
    segs = paint_segments(xy, np.zeros(2), min_count=1)
    assert len(segs) == 1 and abs(math.hypot(segs[0][2] - segs[0][0], segs[0][3] - segs[0][1]) - 3.5) < 0.2
    # a light spot half a metre beyond the end of a stripe does not make the stripe longer
    spot = np.stack(np.meshgrid(np.arange(5.0, 5.2, 0.05), np.arange(0.75, 0.9, 0.05)), axis=-1).reshape(-1, 2)
    segs = paint_segments(np.concatenate([xy, spot]), np.zeros(2), min_count=1)
    assert len(segs) == 1 and max(segs[0][0], segs[0][2]) < 4.6
    # Paint that is nearly worn away, 1.4 times lighter than the road: the rule alone does not see
    # it. It does where a network that labels the image sees a marking, and only there: a label
    # on bare road makes no paint.
    faint = (255.0 * (np.where(image[..., 0] > 150, 1.4, 1.0) * 0.16) ** (1.0 / 2.2) + 0.5).astype(np.uint8)
    faint = np.repeat(faint[..., None], 3, axis=2)
    assert len(rig._paint(faint, flat, np.array([0.0, 0.0, h]), R.astype(np.float32), 0.0, 5.0)[0]) == 0
    labels = np.where(image[..., 0] > 150, MARKING, 0).astype(np.uint8)
    xy, _, _ = rig._paint(faint, flat, np.array([0.0, 0.0, h]), R.astype(np.float32), 0.0, 5.0, labels)
    assert len(xy) > 40 and np.abs(xy[:, 1] - 0.8).max() < 0.10 and xy[:, 0].max() < 2.7, "the lit part of the stripe"
    wrong = np.roll(labels, 60, axis=1)                    # the label beside the stripe
    assert len(rig._paint(faint, flat, np.array([0.0, 0.0, h]), R.astype(np.float32), 0.0, 5.0, wrong)[0]) == 0
    print("paint in sun and shade, shadow edges, light spots, faint paint with a network's hint: ok")


def test_evidence_in_seconds():
    """A line is believed after the same time, however often the perception runs."""
    when = {}
    for hz in (2.5, 5.0, 10.0, 30.0):
        det = (0.0, 0.0, 0.0, 2.0, 5.0, 1.0 / hz)
        lines, n = LineMap(), 0
        while not lines.markers():
            lines.update([det], n / hz)
            n += 1
        when[hz] = (n - 1) / hz
    assert all(0.35 <= t <= 0.41 for t in when.values()), when        # 0.4 s at every rate that divides it
    lines = LineMap()                                    # three cameras seeing it at one instant are one sighting
    for _ in range(3):
        lines.update([det, det, det], 0.0)
    assert lines.tracks[0].watched == 0.0 and not lines.markers()
    lines.update([det], 5.0)                             # out of sight for a while: the gap counts for half a second
    assert lines.tracks[0].watched == LineTrack.GAP

    # the grid: 1.2 s of looking at a wall 6 m ahead, with ground that is sure up to 4 m and
    # probable up to the wall, gives the same map at every rate
    ang = np.radians(np.arange(-20.0, 20.0, 0.5))
    wall, sure = np.full(len(ang), 6.0), np.full(len(ang), 4.0)
    maps, shown = {}, {}
    for hz in (2.5, 5.0, 10.0, 30.0):
        g = GridMap((0.0, -5.0, 12.0, 5.0))
        trk = LineTrack((0.0, 0.0, 0.0, 2.0, 8.0, 1.0 / hz), 0.0, keep=True)
        for n in range(int(round(1.2 * hz))):
            g.update((0.0, 0.0), ang, wall, sure, wall - 0.3, dt=1.0 / hz)
            if n:
                trk.add((0.0, 0.0, 0.0, 2.0, 8.0, 1.0 / hz), n / hz)
        maps[hz] = (g.occupied(), g.clear(), g.blocked())
        shown[hz] = trk.seen[5]
        assert g.occupied().any() and g.clear().any() and g.far_ground().any()
    for hz in (5.0, 10.0, 30.0):
        assert all(np.array_equal(a, b) for a, b in zip(maps[hz], maps[2.5])), "the map depends on the rate (%g Hz)" % hz
        assert abs(shown[hz] - shown[2.5]) < 1e-6 * shown[2.5], "the memory of a line depends on the rate"
    # one frame is never enough, however long it is said to stand for, and neither is 0.3 s of frames
    for dt, n in ((5.0, 1), (0.1, 3), (1.0 / 30.0, 9)):
        g = GridMap((0.0, -5.0, 12.0, 5.0))
        for _ in range(n):
            g.update((0.0, 0.0), ang, wall, sure, wall - 0.3, dt=dt)
        assert not g.occupied().any() and not g.clear().any() and g.seen_free().any(), (dt, n)
    print("evidence in seconds, not in frames: ok")


def test_lines_in_pieces():
    """Collinear pieces of a line are joined over a gap, and nothing else is."""
    def piece(x, y0, y1, watched=0.6):
        t = types.SimpleNamespace(c=np.array([x, 0.5 * (y0 + y1)]), d=np.array([0.0, 1.0]), length=abs(y1 - y0), watched=watched)
        t.ends = lambda: (np.array([x, y0]), np.array([x, y1]))
        return t

    tick = [piece(7.2, -4.23, -3.77), piece(7.21, -2.73, -1.77)]           # a tick with a metre missing in the middle
    other = [piece(0.0, -4.2, -1.8), piece(7.2, 3.5, 9.0), piece(7.6, -3.6, -2.9)]
    out = join_collinear(tick + other)                                 # across the lane and 40 cm to the side: not joined
    assert len(out) == 4
    joined = [t for t in out if isinstance(t, JoinedLine)]
    assert len(joined) == 1 and abs(joined[0].length - 2.46) < 0.01 and joined[0].watched == 0.6
    assert abs(joined[0].c[0] - 7.207) < 0.005 and abs(joined[0].c[1] + 3.0) < 0.01
    print("lines seen in pieces: ok")


def test_half_seen_stall():
    """A stall of which one line is seen whole and the other only in part, in a 60 degree lot: the
    ends of the lines lie on a line along the lane, 1.56 m apart lengthwise."""
    u = np.array([0.5, -math.sqrt(0.75)])                 # into the stalls, from a lane along +x

    def line(x, far):                                     # from the lane edge at y = -2.8 to 'far' along it
        t = types.SimpleNamespace(c=np.array([x, -2.8]) + 0.5 * far * u, d=u, length=far, watched=2.0)
        t.ends = lambda: (np.array([x, -2.8]), np.array([x, -2.8]) + far * u)
        return t

    old = (EGO.rear, EGO.front, EGO.half_width, EGO.length)
    EGO.rear, EGO.front, EGO.half_width, EGO.length = 1.0, 3.8, 0.9, 4.8
    try:
        trail = [np.array([-8.0, 0.0]), np.array([-4.8, 0.0])]
        grid = GridMap((-10.0, -12.0, 12.0, 4.0))
        for whole, part in ((0.0, 3.12), (3.12, 0.0)):   # the half-seen line on either side of the whole one
            slots = find_slots([line(whole, 6.0), line(part, 3.4)], trail, grid, True)
            assert len(slots) == 1 and slots[0].kind == "angled", [s.kind for s in slots]
            mouth = slots[0].corners[:2]
            assert np.abs(mouth[:, 1] + 2.8).max() < 0.05, "the stall must start at the lane edge: %s" % mouth.round(2)
            assert abs(sorted(mouth[:, 0])[0]) < 0.05 and abs(sorted(mouth[:, 0])[1] - 3.12) < 0.05
    finally:
        EGO.rear, EGO.front, EGO.half_width, EGO.length = old
    print("a half-seen stall in an angled lot: ok")


def test_stalls_from_their_row():
    """Stalls of which a camera sees little: taken from the row they stand in."""
    def line(x, y0, y1, watched=2.0):                     # along -y, from y0 (at the lane) to y1
        t = types.SimpleNamespace(c=np.array([x, 0.5 * (y0 + y1)]), d=np.array([0.0, -1.0]), length=y0 - y1, watched=watched)
        t.ends = lambda: (np.array([x, y0]), np.array([x, y1]))
        return t

    old = (EGO.rear, EGO.front, EGO.half_width, EGO.length)
    EGO.rear, EGO.front, EGO.half_width, EGO.length = 1.0, 3.8, 0.9, 4.8
    try:
        trail = [np.array([-8.0, 0.0]), np.array([-4.0, 0.0])]
        grid = GridMap((-12.0, -14.0, 24.0, 4.0))
        # perpendicular stalls 2.7 m wide that open at y = -3.5, most of them with cars in them:
        # of their lines only the first 0.6 m shows. One line further on is seen over 3 m.
        stubs = [line(2.7 * k, -3.5, -4.1) for k in range(5)]
        slots = find_slots(stubs + [line(13.5, -3.5, -6.5)], trail, grid, True)
        between = [s for s in slots if s.center[0] < 10.8]
        assert len(between) == 4 and all(s.kind == "perpendicular" for s in between), [(s.kind, s.center.round(1)) for s in slots]
        for s in between:           # the car's middle 2.75 m in from the mouth, between the two stubs
            assert abs(s.center[1] + 3.5 + 2.75) < 0.05 and abs((s.center[0] - 1.35) % 2.7) < 0.05, s.center
        assert find_slots(stubs, trail, grid, True) == [], "stubs alone do not say which way a stall points"
        # a line that has lost its first 1.2 m: the stall starts where the row does all the same
        worn = [line(0.0, -3.5, -9.0), line(2.7, -4.7, -9.0), line(5.4, -3.5, -9.0), line(8.1, -3.5, -9.0)]
        slots = find_slots(worn, trail, grid, True)
        first = min(slots, key=lambda s: s.center[0])
        assert abs(first.center[1] + 3.5 + 2.75) < 0.05, "0.4 m too deep without the row: %s" % first.center.round(2)
        # a kerb that a camera network saw 5.0 m in: the car ends 0.4 m short of it
        X = np.arange(-1.0, 10.0, 0.1)
        grid.add_kerb(np.stack([np.repeat(X, 3), np.tile([-8.5, -8.6, -8.7], len(X))], axis=1), 0.4)
        capped = min(find_slots(worn, trail, grid, True), key=lambda s: s.center[0])
        assert abs(capped.center[1] + 8.5 - 0.4 - 0.5 * EGO.length) < 0.11, capped.center.round(2)
        # and one that lies at a slant, 0.58 m further in per metre along the lane: what counts is
        # where it is at the side of the car that it is nearer to
        grid = GridMap((-12.0, -14.0, 24.0, 4.0))
        grid.add_kerb(np.stack([X, -8.5 - 0.58 * (X - 1.35)], axis=1), 0.4)
        capped = min(find_slots(worn, trail, grid, True), key=lambda s: s.center[0])
        assert abs(capped.center[1] + 8.5 - 0.58 * (EGO.half_width + 0.1) - 0.4 - 0.5 * EGO.length) < 0.15, capped.center.round(2)
        # parallel stalls along a kerb, 6.5 m long: two ends of ticks and the kerb behind them
        grid = GridMap((-12.0, -14.0, 24.0, 4.0))
        ticks = [line(6.5 * k, -1.75, -2.3) for k in range(4)]
        assert find_slots(ticks, trail, grid, True) == [], "without a kerb these could be anything"
        X = np.arange(-1.0, 21.0, 0.1)
        grid.add_kerb(np.stack([np.repeat(X, 2), np.tile([-4.3, -4.4], len(X))], axis=1), 0.4)
        slots = find_slots(ticks, trail, grid, True)
        assert len(slots) == 3 and all(s.kind == "parallel" for s in slots), [(s.kind, s.center.round(1)) for s in slots]
        assert all(abs(s.center[1] + 1.75 + 1.25) < 0.05 for s in slots)
    finally:
        EGO.rear, EGO.front, EGO.half_width, EGO.length = old
    print("stalls taken from their row: ok")


def test_stall_with_one_line():
    """A stall of which one line was found: the row says where the other is, a parked car that it is one."""
    def line(x, y1, y0=-3.5, d=(0.0, -1.0)):             # from (x, y0) at the lane, along d, as far in as y1
        d = np.array(d)
        far = np.array([x, y0]) + d * (y1 - y0) / d[1]
        t = types.SimpleNamespace(c=0.5 * (np.array([x, y0]) + far), d=d, length=float(np.hypot(*(far - (x, y0)))), watched=2.0)
        t.ends = lambda: (np.array([x, y0]), far)
        return t

    def fill(layer, x0, x1, y0, y1, keep=lambda x, y: True):
        X, Y = np.meshgrid(np.arange(x0, x1, 0.1) + 0.05, np.arange(y0, y1, 0.1) + 0.05)
        ix, iy, ok = grid.cells(X, Y)
        ok = ok & keep(X, Y)
        layer[iy[ok], ix[ok]] = 1.0

    old = (EGO.rear, EGO.front, EGO.half_width, EGO.length)
    EGO.rear, EGO.front, EGO.half_width, EGO.length = 1.0, 3.8, 0.9, 4.8
    try:
        trail = [np.array([-8.0, 0.0]), np.array([-4.0, 0.0])]
        # stalls 2.7 m wide: cars in three of them, of whose lines only the ends show, then a free
        # one of which the line at x = 8.1 is gone, then another car
        lines = [line(0.0, -4.1), line(2.7, -4.1), line(5.4, -4.1), line(10.8, -6.5)]
        for cars in (True, False):
            grid = GridMap((-12.0, -14.0, 24.0, 4.0))
            fill(grid.free, -2.0, 16.0, -3.5, 3.0)
            fill(grid.free, 8.1, 10.8, -9.0, -3.5)
            for x in (0.0, 2.7, 5.4, 10.8) if cars else ():
                fill(grid.hits, x + 0.4, x + 2.3, -8.5, -4.0)
            found = [s for s in find_slots(lines, trail, grid, True) if s.status == "free"]
            if cars:
                assert len(found) == 1 and found[0].by_row and found[0].kind == "perpendicular", [(s.kind, s.center.round(1)) for s in found]
                assert np.abs(found[0].center - (9.45, -6.25)).max() < 0.06, found[0].center
            else:               # free ground beside a line, and no car anywhere: not a stall
                assert not found, [(s.kind, s.center.round(1)) for s in found]
        # The car before the stall is a wide one, and of the stall's ground only a narrow wedge
        # was seen past its corner: less than half of the first 2.5 m. But the first 1.2 m, where
        # the front of a parked car would be, is seen empty nearly all across. Free.
        for seen, free in ((lambda x, y: x > 8.1 + 1.1 * (-3.5 - y), True),
                           (lambda x, y: (x > 8.1 + 1.1 * (-3.5 - y)) & (y < -4.3), False)):    # (the first 0.8 m not seen)
            grid = GridMap((-12.0, -14.0, 24.0, 4.0))
            fill(grid.free, -2.0, 16.0, -3.5, 3.0)
            fill(grid.free, 8.1, 10.8, -5.7, -3.5, seen)
            fill(grid.free, 10.2, 10.8, -6.1, -3.5)
            for x in (0.0, 2.7, 5.4, 10.8):
                fill(grid.hits, x + 0.4, x + 2.3, -8.5, -4.0)
            found = [s for s in find_slots(lines, trail, grid, True) if s.status == "free"]
            assert len(found) == free, [(s.kind, s.center.round(1)) for s in found]

        # Angled stalls, 60 degrees, 3.1 m apart along the lane. The stall next to a staggered one
        # starts 1.55 m further out, and so does the car in it: of that car the cameras saw the
        # front only. It has to count as the neighbour, or nothing says the free place is a stall.
        d = (0.5, -math.sqrt(0.75))
        lines = [line(x, -4.7, -2.8, d) for x in (0.0, 3.1, 6.2, 9.3)]                    # (the line at 12.4 is gone)
        lines += [line(x, 4.7, 2.8, (0.5, math.sqrt(0.75))) for x in np.arange(0.0, 18.0, 3.1)]      # the row across the lane
        grid = GridMap((-12.0, -14.0, 24.0, 8.0))
        fill(grid.free, -2.0, 20.0, -2.8, 2.8)
        fill(grid.free, 9.0, 18.0, -8.0, -2.8, lambda x, y: (x - 9.3 > 0.5 * (-2.8 - y) / 0.866) & (x - 12.4 < 0.5 * (-2.8 - y) / 0.866))
        for x in (0.0, 3.1, 6.2):                           # the front of the car in each stall before
            m = np.array([x + 1.55, -2.8]) + 1.3 * np.array(d)
            fill(grid.hits, m[0] - 0.6, m[0] + 0.6, m[1] - 0.6, m[1] + 0.6)
        found = [s for s in find_slots(lines, trail, grid, True) if s.status == "free" and s.center[1] < 0.0]
        assert len(found) == 1 and found[0].by_row and found[0].kind == "angled", [(s.kind, s.center.round(1)) for s in found]
        assert sum(found[0].neighbors) == 1 and abs((found[0].center - (10.85, -2.8)) @ (0.866, 0.5)) < 0.06, (found[0].neighbors, found[0].center)

        # Parallel stalls 7.2 m long along a kerb, the tick at x = 28.8 gone, a van before the free
        # one. Beside the first tick lies a bit of something else that makes a narrow stall with
        # it. The row is one of parallel stalls all the same: its ticks are a car's length apart.
        lines = [line(x, -4.2, -1.8) for x in (0.0, 7.2, 14.4, 21.6, 36.0)] + [line(2.5, -4.3, -3.8)]
        grid = GridMap((-12.0, -14.0, 50.0, 4.0))
        fill(grid.free, -2.0, 45.0, -1.8, 3.0)
        fill(grid.free, 28.8, 36.0, -4.2, -1.8)
        for x in (0.0, 7.2, 14.4, 21.6):
            fill(grid.hits, x + 0.8, x + 6.4, -4.0, -2.1)
        X = np.arange(-1.0, 45.0, 0.1)
        grid.add_kerb(np.stack([np.repeat(X, 2), np.tile([-4.3, -4.4], len(X))], axis=1), 0.4)
        found = [s for s in find_slots(lines, trail, grid, True) if s.status == "free"]
        assert len(found) == 1 and found[0].by_row and found[0].kind == "parallel", [(s.kind, s.center.round(1)) for s in found]
        assert np.abs(found[0].center - (32.4, -3.05)).max() < 0.06, found[0].center
    finally:
        EGO.rear, EGO.front, EGO.half_width, EGO.length = old
    print("a stall with one line found: ok")


def test_parallel_stall_and_kerb():
    """A parallel stall is aligned with the kerb behind it, and not with what else was hit there."""
    def tick(x):                                          # from the lane edge at y = -1.75 to the kerb
        t = types.SimpleNamespace(c=np.array([x, -3.0]), d=np.array([0.0, -1.0]), length=2.5, watched=2.0)
        t.ends = lambda: (np.array([x, -1.75]), np.array([x, -4.25]))
        return t

    old = (EGO.rear, EGO.front, EGO.half_width, EGO.length)
    EGO.rear, EGO.front, EGO.half_width, EGO.length = 1.0, 3.8, 0.9, 4.8
    try:
        trail = [np.array([0.0, 0.0]), np.array([6.0, 0.0])]
        grid = GridMap((-5.0, -10.0, 30.0, 4.0))
        X = np.arange(13.0, 23.0, 0.1)
        ix, iy, _ = grid.cells(np.repeat(X, 3), np.tile([-4.30, -4.40, -4.50], len(X)))     # the kerb, its face at y = -4.25
        grid.hits[iy, ix] = 1.0
        slot = find_slots([tick(14.4), tick(21.6)], trail, grid, True)[0]
        angle = math.degrees(math.atan2(slot.along[1], slot.along[0]))
        assert slot.kind == "parallel" and abs((angle + 90.0) % 180.0 - 90.0) < 0.5
        assert abs(slot.center[1] - (-4.25 + 0.30 + EGO.half_width)) < 0.08, slot.center
        # The kerb seen only as far as x = 19.5, and the corner of the car parked behind the stall,
        # 0.5 m nearer than the kerb. A plain fit to all of that comes out 2.8 degrees off.
        grid = GridMap((-5.0, -10.0, 30.0, 4.0))
        for xs, ys in ((np.arange(13.0, 19.5, 0.1), [-4.30, -4.40, -4.50]), (np.arange(20.4, 21.4, 0.1), [-3.70, -3.80, -3.90])):
            ix, iy, _ = grid.cells(np.repeat(xs, 3), np.tile(ys, len(xs)))
            grid.hits[iy, ix] = 1.0
        again = find_slots([tick(14.4), tick(21.6)], trail, grid, True)[0]
        angle = math.degrees(math.atan2(again.along[1], again.along[0]))
        assert abs((angle + 90.0) % 180.0 - 90.0) < 0.5 and np.hypot(*(again.center - slot.center)) < 0.05, (angle, again.center)
    finally:
        EGO.rear, EGO.front, EGO.half_width, EGO.length = old
    print("a parallel stall and the kerb behind it: ok")


def test_own_pose():
    """What the car believes about its own pose, and the road it stands on."""
    path = [(0.0, 0.0, 0.0)]                                             # 40 s at 1 m/s along a gentle curve
    for k in range(399):
        x, y, th = path[-1]
        path.append((x + 0.1 * math.cos(th + 0.001), y + 0.1 * math.sin(th + 0.001), th + 0.002))
    exact = Localization(path[0], np.random.default_rng(0), "gps", 0.0)
    assert all(exact.update(q, 0.1 * k) == q for k, q in enumerate(path)) and exact.tilt is None
    assert exact.as_believed([(0.1, (1.0, 2.0), np.zeros(3), np.zeros(3), np.zeros(3))], [(0, 0, 1, 1, 5.0, 0.1)])[1] == [(0, 0, 1, 1, 5.0, 0.1)]
    err, step = [], []
    for seed in range(40):
        loc, last = Localization(path[0], np.random.default_rng(seed), "gps", 1.0), None
        for k, q in enumerate(path):
            e = np.array(loc.update(q, 0.1 * k)[:2]) - q[:2]
            if last is not None:
                step.append(np.hypot(*(e - last)))
            err.append(np.hypot(*e))
            last = e
        assert abs(math.degrees(wrap(loc.pose[2] - path[-1][2]))) < 1.5 and max(abs(math.degrees(a)) for a in loc.tilt) < 1.0
    # off by about 14 cm in all (10 cm per axis), never wildly, and smooth from one moment to the next
    assert 0.08 < np.mean(err) < 0.20 and max(err) < 0.6 and max(step) < 0.02, (np.mean(err), max(err), max(step))
    drift = []
    for seed in range(40):           # dead reckoning: nothing at the start, and it grows
        loc = Localization(path[0], np.random.default_rng(seed), "odometry", 1.0)
        off = [np.hypot(*(np.array(loc.update(q, 0.1 * k)[:2]) - q[:2])) for k, q in enumerate(path)]
        drift.append((off[0], off[100], off[-1]))
    drift = np.array(drift).mean(axis=0)
    assert drift[0] == 0.0 and 0.01 < drift[1] < 0.15 and drift[2] > 2.5 * drift[1], drift
    # a sensor reading taken at the true pose lands where the car thinks it is: same place relative to the car
    loc = Localization((3.0, 1.0, 0.5), np.random.default_rng(1), "gps", 3.0)
    loc.update((3.0, 1.0, 0.5), 0.0)
    scans, dets = loc.as_believed([(0.1, (3.0, 1.0), np.array([0.5]), np.array([2.0]), np.array([2.0]))], [(4.0, 1.0, 5.0, 1.0, 1.5, 0.1)])
    assert np.allclose(scans[0][1], loc.pose[:2]) and abs(scans[0][2][0] - loc.pose[2]) < 1e-12
    assert abs(math.hypot(dets[0][2] - dets[0][0], dets[0][3] - dets[0][1]) - 1.0) < 1e-9
    assert abs(math.hypot(dets[0][0] - loc.pose[0], dets[0][1] - loc.pose[1]) - 1.0) < 1e-9

    flat = Ground((0.0, -10.0, 30.0, 10.0))
    assert flat.height(3.0, 4.0) == 0.0 and flat.slope(3.0, 4.0) == (0.0, 0.0)
    g = Ground((0.0, -10.0, 30.0, 10.0), 0.015, seed=3)
    x, y = np.meshgrid(np.arange(-20.0, 50.0, 0.37), np.arange(-30.0, 30.0, 0.37))
    h = g.height(x, y)
    assert abs(np.abs(h).max() - 0.015) < 0.002 and abs(h.mean()) < 0.004
    gx, gy = np.gradient(h, 0.37, axis=1), np.gradient(h, 0.37, axis=0)
    assert np.hypot(gx, gy).max() < 0.02, "slopes of at most 2 percent"            # waves of 6 m and longer
    sx, sy = g.slope(10.0, 2.0)
    assert abs(sx - (g.height(11.5, 2.0) - g.height(8.5, 2.0)) / 3.0) < 1e-12 and abs(sy) < 0.02
    assert not np.array_equal(h, Ground((0.0, -10.0, 30.0, 10.0), 0.015, seed=4).height(x, y))
    print("the car's own pose and the uneven road: ok")


def test_worn_paint():
    """No line is perfect: the pieces of a line, and the images of worn paint."""
    from parking.world import write_png
    lines = [(float(k), 0.0, float(k), 5.5, "white") for k in range(600)]
    pieces = lay(lines, 1.0, 3)
    state = np.array([q[6] for q in pieces])
    share = [(state == k).mean() for k in range(len(STATES))]
    assert 0.5 < share[0] < 0.7 and 0.2 < share[1] < 0.35 and 0.07 < share[2] < 0.2, share       # (one piece in seven is a state worse than its line)
    assert 0.85 * 600 * 11 < len(pieces) < 0.96 * 600 * 11, "some pieces are gone, most are there"
    off = np.array([q[0] - round(q[0]) for q in pieces])
    assert 0.002 < np.abs(off).max() <= 0.0101 and all(0.09 < q[4] < 0.125 for q in pieces)       # wanders, never wider than new
    assert max(q[6] for q in lay(lines, 0.0, 3)) <= 1, "with no extra wear no line is nearly gone"
    files = paint_textures(write_png)
    assert len(files) == 2 * len(STATES) * 3 * 3 and all(os.path.getsize(f) > 300 for f in files.values())
    print("worn paint: ok")


def test_surfaces():
    """The road surface images exist, are PNG files and have the reflectance of asphalt."""
    files = surface_textures()
    raw = open(files["asphalt"], "rb").read()
    assert raw[:8] == b"\x89PNG\r\n\x1a\n" and raw[12:16] == b"IHDR"
    w, h = int.from_bytes(raw[16:20], "big"), int.from_bytes(raw[20:24], "big")
    assert (w, h) == (1024, 1024)
    import zlib
    k = raw.index(b"IDAT")
    data = zlib.decompress(raw[k + 4:k + 4 + int.from_bytes(raw[k - 4:k], "big")])
    px = np.frombuffer(data, np.uint8).reshape(h, 1 + 3 * w)[:, 1:].reshape(h, w, 3)
    lin = (px[..., 0] / 255.0) ** 2.2
    assert 0.13 < lin.mean() < 0.19 and 0.02 < lin.std() < 0.06                          # dark grey, and not flat
    print("road surface images: ok")


if __name__ == "__main__":
    test_reeds_shepp()
    test_mpc_solver()
    test_footprint_and_distance()
    test_sensor_geometry()
    test_mapping()
    test_pictures()
    test_depth_from_images()
    test_paint_in_light_and_shade()
    test_evidence_in_seconds()
    test_lines_in_pieces()
    test_half_seen_stall()
    test_stalls_from_their_row()
    test_stall_with_one_line()
    test_parallel_stall_and_kerb()
    test_own_pose()
    test_worn_paint()
    test_surfaces()
    print("all checks passed")
