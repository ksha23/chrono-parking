#!/usr/bin/env python3
# =============================================================================
# Automated parking in Project Chrono (PyChrono)
#
# A Chrono::Vehicle sedan (full multibody model, TMeasy or Pacejka tires) cruises
# down a parking aisle, builds a map of the stall lines and obstacles from noisy
# simulated perception, decides which stall to take, plans a forward/reverse
# maneuver into it and tracks that plan with model predictive control. One
# window shows four live camera views and a panel with the internals.
#
#   perception : noisy line-segment detections + a noisy 2D range scan
#   mapping    : line tracks (total least squares) + occupancy grid
#   decision   : stalls are inferred from pairs of tracked lines, classified
#                free / occupied / unknown, and scored
#   planning   : Hybrid A* with Reeds-Shepp and arc-line analytic expansions
#   control    : linear MPC on the steering (constrained QP, solved exactly) with
#                the steering gain identified online, and a PI loop on speed
#
# Nothing about the car is hard-coded: its geometry and steering limit are read
# from the Chrono model at start-up. The design is documented in docs/.
#
# Examples (any Python with PyChrono's vehicle and irrlicht modules):
#
#   python parking_sim.py                                  perpendicular, car on each side
#   python parking_sim.py --type angled --cars none        60 deg stalls, empty lot
#   python parking_sim.py --type parallel --cars both      parallel park between two cars
#   python parking_sim.py --type perpendicular --cars left --park forward
#   python parking_sim.py --tour                           a set of scenarios back to back
#   python parking_sim.py --target drag                    place the target box yourself
#   python parking_sim.py --headless --seed 7 --noise 2    no window, prints the result
#
# "python parking_sim.py --help" lists every option.
# =============================================================================

import argparse
import collections
import glob
import heapq
import math
import os
import subprocess
import sys
import threading
import time

import numpy as np


def _reexec_in_chrono_env():
    """PyChrono is not importable here: look for a conda env that has it."""
    if os.environ.get("PARKING_SIM_REEXEC"):
        return
    roots = []
    for var in ("CONDA_EXE", "MAMBA_EXE"):
        exe = os.environ.get(var)
        if exe:
            roots.append(os.path.dirname(os.path.dirname(exe)))
    roots += ["/opt/homebrew/Caskroom/miniconda/base", "/opt/homebrew/anaconda3",
              os.path.expanduser("~/miniconda3"), os.path.expanduser("~/anaconda3"),
              os.path.expanduser("~/miniforge3"), os.path.expanduser("~/mambaforge")]
    found = []
    for root in dict.fromkeys(roots):
        for f in glob.glob(os.path.join(root, "envs", "*", "lib", "python*", "site-packages",
                                        "pychrono", "vehicle.py")):
            if os.path.exists(os.path.join(os.path.dirname(f), "irrlicht.py")):
                env = f.split(os.sep + "lib" + os.sep)[0]
                py = os.path.join(env, "bin", "python")
                if os.path.exists(py):
                    found.append((os.path.getmtime(f), py))
    if found:
        py = max(found)[1]
        print("[parking] PyChrono not found in this interpreter, re-running with %s" % py)
        os.environ["PARKING_SIM_REEXEC"] = "1"
        os.execv(py, [py] + sys.argv)


try:
    import pychrono as chrono
    import pychrono.vehicle as veh
except ImportError:
    _reexec_in_chrono_env()
    sys.exit("PyChrono with the vehicle and irrlicht modules is required (see docs/ for the setup)")

# =============================================================================
# Ego vehicle. Nothing about the car is hard-coded: its geometry and steering
# limit are read from the Chrono model when the world is built, and its
# steering response is identified online while it drives (see MpcTracker).
# All planning and control is referenced to the centre of the rear axle.
# =============================================================================

class Ego:
    def __init__(self):
        self.wheelbase = self.rear = self.front = self.half_width = None
        self.ref_to_rear = self.length = self.center = self.kappa = self.radius = None

    def read(self, car):
        """Query the Chrono vehicle: wheelbase, axle position, body outline, steering limit."""
        body = car.GetChassisBody()
        frame = body.GetFrameRefToAbs()
        rear = [frame.TransformPointParentToLocal(car.GetSpindlePos(car.GetNumberAxles() - 1, side)).x
                for side in (veh.LEFT, veh.RIGHT)]
        self.wheelbase = car.GetWheelbase()
        self.ref_to_rear = -0.5 * (rear[0] + rear[1])      # chassis reference frame -> rear axle
        # body outline: the convex hull the chassis collides with, in the chassis reference frame
        pts = []
        model = body.GetCollisionModel()
        for i in range(model.GetNumShapes()):
            hull = chrono.CastToChCollisionShapeConvexHull(model.GetShapeInstance(i).shape)
            pts += [(q.x, q.y) for q in hull.GetPoints()]
        pts = np.array(pts)
        self.rear = -self.ref_to_rear - pts[:, 0].min()     # rear axle -> rear bumper
        self.front = pts[:, 0].max() + self.ref_to_rear     # rear axle -> front bumper
        self.half_width = float(np.abs(pts[:, 1]).max())
        self.length = self.front + self.rear
        self.center = 0.5 * (self.front - self.rear)       # rear axle -> middle of the body
        # kinematic (bicycle) curvature at the model's maximum steering angle
        self.kappa = math.tan(car.GetMaxSteeringAngle()) / self.wheelbase
        self.radius = 1.0 / self.kappa


EGO = Ego()

STEP = 2e-3                    # simulation step [s]
CONTROL_DT = 0.02              # controller period [s]
PERCEPTION_DT = 0.1            # perception / mapping period [s]

V_SEARCH = 2.2                 # cruise speed while looking for a stall [m/s]
V_FWD = 1.4                    # maneuver speed, forward [m/s]
V_REV = 1.0                    # maneuver speed, reverse [m/s]
STEER_RATE = 1.3               # limit on the steering input rate [1/s] (lock to lock in ~1.5 s)


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rect_poly(x, y, yaw, x0, x1, hw):
    """Corners of a body-frame rectangle x in [x0, x1], y in [-hw, hw] placed at (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(x + px * c - py * s, y + px * s + py * c)
                     for px, py in ((x0, -hw), (x1, -hw), (x1, hw), (x0, hw))])


def ego_poly(pose, margin=0.0):
    return rect_poly(pose[0], pose[1], pose[2], -EGO.rear - margin, EGO.front + margin,
                     EGO.half_width + margin)


def _separated(a, b):
    for poly in (a, b):
        e = np.roll(poly, -1, axis=0) - poly
        n = np.stack([-e[:, 1], e[:, 0]], axis=1)
        pa, pb = a @ n.T, b @ n.T
        if np.any(pa.max(0) < pb.min(0)) or np.any(pb.max(0) < pa.min(0)):
            return True
    return False


def _pts_to_edges(p, poly):
    a = poly
    d = np.roll(poly, -1, axis=0) - poly
    ap = p[:, None, :] - a[None, :, :]
    t = np.clip((ap * d[None]).sum(2) / np.maximum((d * d).sum(1)[None], 1e-12), 0.0, 1.0)
    q = ap - t[..., None] * d[None]
    return np.sqrt((q * q).sum(2)).min()


def poly_distance(a, b):
    """Distance between two convex polygons (0 if they overlap)."""
    if not _separated(a, b):
        return 0.0
    return float(min(_pts_to_edges(a, b), _pts_to_edges(b, a)))


def footprint_hits(poses, pts, margin):
    """poses (N,3), pts (M,2): for every pose, is any point inside the (inflated) ego footprint?"""
    out = np.zeros(len(poses), dtype=bool)
    if len(poses) == 0 or len(pts) == 0:
        return out
    px, py = pts[:, 0].astype(np.float32), pts[:, 1].astype(np.float32)
    for i in range(0, len(poses), 16):       # a few poses at a time: bounded working memory
        q = poses[i:i + 16]
        c, s = np.cos(q[:, 2])[:, None], np.sin(q[:, 2])[:, None]
        dx, dy = px[None, :] - q[:, 0, None], py[None, :] - q[:, 1, None]
        lx = dx * c + dy * s
        ly = dy * c - dx * s
        out[i:i + 16] = ((lx > -EGO.rear - margin) & (lx < EGO.front + margin) &
                         (np.abs(ly) < EGO.half_width + margin)).any(axis=1)
    return out


# =============================================================================
# Scenarios: ground truth layout of painted lines, stalls, parked cars and curbs
# =============================================================================

# Parked-car models, built from the vehicle meshes that ship with Chrono. Only the wheel hub
# positions (x, y, z of the front and rear left hubs, from Chrono's vehicle definitions) are
# listed here; the footprint and ride height come from the meshes themselves.
PARKED = {
    "audi": dict(mesh="audi/audi_chassis_%s.obj", wheels=((1.441, 0.798, 0.13), (-1.480, 0.798, 0.13)),
                 rim="audi/audi_rim.obj", tire="audi/audi_tire.obj",
                 colors=("black", "blue", "grey", "white")),
    "suv": dict(mesh="Nissan_Patrol/suv_chassis_%s.obj", wheels=((0.005, 0.960, 0.094), (-3.264, 1.010, 0.054)),
                rim="Nissan_Patrol/suv_rim.obj", tire="Nissan_Patrol/suv_tire.obj",
                colors=("brown", "darkgrey", "red", "white")),
    "van": dict(mesh="VW_microbus/van_chassis_%s.obj", wheels=((-0.040, 0.698, -0.026), (-2.400, 0.728, 0.0)),
                rim="VW_microbus/van_rim.obj", tire="VW_microbus/van_tire.obj",
                colors=("cream", "darkgrey", "lightgrey", "yellow")),
}
_PARKED_GEOMETRY = {}


def parked_model(name):
    """A parked-car model with its footprint (x0, x1, hw in the mesh frame) and ride height z."""
    m = _PARKED_GEOMETRY.get(name)
    if m is None:
        m = dict(PARKED[name], x0=-2.45, x1=2.45, hw=1.05, z=0.25)
        data = chrono.GetChronoDataPath() + "vehicle/"
        body, tire = data + m["mesh"] % m["colors"][0], data + m["tire"]
        if os.path.exists(body) and os.path.exists(tire):
            box = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(body, False, False).GetBoundingBox()
            wheel = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(tire, False, False).GetBoundingBox()
            hub = sum(w[2] for w in m["wheels"]) / len(m["wheels"])
            m.update(x0=box.min.x, x1=box.max.x, hw=max(-box.min.y, box.max.y),
                     z=0.5 * (wheel.max.z - wheel.min.z) - hub)       # tires resting on the ground
        _PARKED_GEOMETRY[name] = m
    return m


STALL_WIDTH = 2.7
STALL_DEPTH = 5.5
PARALLEL_LENGTH = 7.2
PARALLEL_DEPTH = 2.5


class Scenario:
    def __init__(self, name):
        self.name = name
        self.lines = []      # (x1, y1, x2, y2, colour)
        self.stalls = []     # dict(corners, center, u_in, kind, occupied)
        self.cars = []       # dict(model, color, cx, cy, yaw, poly)
        self.curbs = []      # (cx, cy, lx, ly)  axis-aligned raised kerbs
        self.pads = []       # (cx, cy, lx, ly, rgb)  visual-only ground patches
        self.start = (0.0, 0.0, 0.0)
        self.route_end = 0.0
        self.bounds = (0.0, 0.0, 1.0, 1.0)

    def add_car(self, rng, cx, cy, yaw, models=("audi", "suv", "van")):
        model = models[int(rng.integers(len(models)))]
        m = parked_model(model)
        color = m["colors"][int(rng.integers(len(m["colors"])))]
        half = 0.5 * (m["x1"] - m["x0"])
        poly = rect_poly(cx, cy, yaw, -half, half, m["hw"])
        self.cars.append(dict(model=model, color=color, cx=cx, cy=cy, yaw=yaw, poly=poly))

    def obstacle_polys(self):
        polys = [c["poly"] for c in self.cars]
        for cx, cy, lx, ly in self.curbs:
            polys.append(rect_poly(cx, cy, 0.0, -0.5 * lx, 0.5 * lx, 0.5 * ly))
        return polys


def _occupancy(cars, n, rng):
    """Which of the n stalls of the target row hold a car, for a --cars preset.
    'left'/'right' are as seen from the driving lane, looking into the stall."""
    if cars == "none":
        return [False] * n
    if cars == "random":
        occ = [bool(rng.random() < 0.7) for _ in range(n)]
        if all(occ):
            occ[int(rng.integers(1, n - 1))] = False
        return occ
    occ = [True] * n
    if cars == "both":
        occ[n // 2] = False
    elif cars == "first":      # target is the first stall of the row
        occ[0] = False
    elif cars == "last":       # target is the last stall of the row
        occ[n - 1] = False
    return occ


def make_lot(angle_deg, cars, side, rng, n=7):
    """Two rows of perpendicular (90 deg) or angled stalls either side of an aisle along +x."""
    scn = Scenario("%s lot" % ("perpendicular" if angle_deg > 80 else "%d deg angled" % angle_deg))
    a = math.radians(angle_deg)
    w = STALL_WIDTH
    aisle = 7.0 if angle_deg > 80 else (5.5 if angle_deg > 52 else 4.8)
    ell = STALL_DEPTH + (w / math.tan(a) if angle_deg < 89 else 0.0)   # painted line length
    pitch = w / math.sin(a)
    depth = ell * math.sin(a)
    x_end = n * pitch + ell * math.cos(a)

    # a neighbour on the left (seen from the aisle) is the next stall along +x for the
    # right-hand row and the previous one for the left-hand row
    preset = cars
    if cars in ("left", "right"):
        after = (cars == "left") == (side < 0)
        preset = "first" if after else "last"

    for sg in (-1, 1):
        u = np.array([math.cos(a), sg * math.sin(a)])
        base = [np.array([i * pitch, sg * 0.5 * aisle]) for i in range(n + 1)]
        for b in base:
            e = b + ell * u
            scn.lines.append((b[0], b[1], e[0], e[1], "white"))
        if sg == side:
            occ = _occupancy(preset, n, rng)
        else:
            occ = [False] * n if cars == "none" else \
                  ([bool(rng.random() < 0.75) for _ in range(n)] if cars == "random" else [True] * n)
        for k in range(n):
            mid = 0.5 * (base[k] + base[k + 1])
            ctr = mid + 0.5 * ell * u
            corners = np.array([base[k], base[k + 1], base[k + 1] + ell * u, base[k] + ell * u])
            scn.stalls.append(dict(corners=corners, center=ctr, u_in=u,
                                   kind="perpendicular" if angle_deg > 80 else "angled",
                                   occupied=occ[k], target_row=(sg == side)))
            if occ[k]:
                nv = np.array([-u[1], u[0]])
                p = ctr + u * rng.uniform(-0.15, 0.15) + nv * rng.uniform(-0.12, 0.12)
                yaw = math.atan2(u[1], u[0]) + math.radians(rng.uniform(-2.0, 2.0))
                if angle_deg > 80 and rng.random() < 0.3:
                    yaw += math.pi          # backed in
                scn.add_car(rng, p[0], p[1], yaw)
        yc = sg * (0.5 * aisle + depth + 0.2 + 0.3)
        scn.curbs.append((0.5 * x_end, yc, x_end + 10.0, 0.6))
        scn.pads.append((0.5 * x_end, sg * (0.5 * aisle + depth + 0.8 + 3.0), x_end + 10.0, 6.0,
                         (0.25, 0.42, 0.22)))

    scn.start = (-14.0, -0.9 if angle_deg > 80 else 0.0, 0.0)
    scn.route_end = n * pitch + 6.0
    ymax = 0.5 * aisle + depth + 0.5
    scn.bounds = (-20.0, -ymax, x_end + 12.0, ymax)
    return scn


def make_street(cars, side, rng, n=5):
    """Kerb-side parallel parking stalls along a two-lane street running along +x."""
    scn = Scenario("parallel street")
    half_lane, L, d = 1.75, PARALLEL_LENGTH, PARALLEL_DEPTH
    sg = side
    u = np.array([0.0, float(sg)])
    for i in range(n + 1):
        scn.lines.append((i * L, sg * half_lane, i * L, sg * (half_lane + d), "white"))
    scn.lines.append((-30.0, -sg * half_lane, n * L + 20.0, -sg * half_lane, "yellow"))

    preset = cars
    if cars in ("left", "right"):
        after = (cars == "left") == (side < 0)
        preset = "first" if after else "last"
    occ = _occupancy(preset, n, rng)
    tgt = occ.index(False) if cars == "both" else None
    for k in range(n):
        ctr = np.array([(k + 0.5) * L, sg * (half_lane + 0.5 * d)])
        corners = np.array([[k * L, sg * half_lane], [(k + 1) * L, sg * half_lane],
                            [(k + 1) * L, sg * (half_lane + d)], [k * L, sg * (half_lane + d)]])
        scn.stalls.append(dict(corners=corners, center=ctr, u_in=u, kind="parallel",
                               occupied=occ[k], target_row=True))
        if occ[k]:
            dx = rng.uniform(-0.3, 0.3)
            if tgt is not None and abs(k - tgt) == 1:
                dx = (tgt - k) * rng.uniform(0.0, 0.3)      # crowd the gap a little
            p = ctr + np.array([dx, sg * rng.uniform(-0.05, 0.08)])
            scn.add_car(rng, p[0], p[1], math.radians(rng.uniform(-1.5, 1.5)))
    x_end = n * L
    for y, wdt in ((sg * (half_lane + d + 0.3), 0.6), (-sg * (3 * half_lane + 0.3), 0.6)):
        scn.curbs.append((0.5 * x_end, y, x_end + 60.0, wdt))
    scn.pads.append((0.5 * x_end, sg * (half_lane + d + 0.6 + 2.0), x_end + 60.0, 4.0, (0.62, 0.62, 0.60)))
    scn.pads.append((0.5 * x_end, -sg * (3 * half_lane + 0.6 + 2.0), x_end + 60.0, 4.0, (0.62, 0.62, 0.60)))
    scn.start = (-16.0, 0.0, 0.0)
    scn.route_end = x_end + 5.0
    lo, hi = sorted((sg * (half_lane + d) , -sg * 3 * half_lane))
    scn.bounds = (-22.0, lo - 0.3, x_end + 14.0, hi + 0.3)
    return scn


def make_scenario(kind, cars, side, angle, seed):
    rng = np.random.default_rng(seed)
    sg = -1 if side == "right" else 1
    if kind == "parallel":
        return make_street(cars, sg, rng)
    return make_lot(90.0 if kind == "perpendicular" else angle, cars, sg, rng)


# =============================================================================
# Simulated perception: noisy stall-line segments and a noisy planar range scan
# =============================================================================

class Perception:
    N_RAYS = 360
    SCAN_RANGE = 16.0
    LINE_RANGE = 12.0

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
        """Returns (origin, ray angles, measured ranges, line detections).
        Ranges are NaN where a ray was dropped and inf where nothing was hit."""
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
        return (ox, oy), ang, r_meas, dets


# =============================================================================
# Mapping: occupancy grid from the scan, line tracks from the line detections
# =============================================================================

class GridMap:
    RES = 0.1

    def __init__(self, bounds):
        self.x0, self.y0 = bounds[0], bounds[1]
        self.nx = int(math.ceil((bounds[2] - bounds[0]) / self.RES))
        self.ny = int(math.ceil((bounds[3] - bounds[1]) / self.RES))
        self.hits = np.zeros((self.ny, self.nx), dtype=np.uint16)
        self.free = np.zeros((self.ny, self.nx), dtype=np.uint16)
        self._t = np.arange(0.1, Perception.SCAN_RANGE, 0.2)

    def cells(self, x, y):
        ix = np.floor((np.asarray(x) - self.x0) / self.RES).astype(int)
        iy = np.floor((np.asarray(y) - self.y0) / self.RES).astype(int)
        ok = (ix >= 0) & (ix < self.nx) & (iy >= 0) & (iy < self.ny)
        return ix, iy, ok

    def update(self, origin, ang, r):
        seen = np.isfinite(r)
        hx, hy = origin[0] + r[seen] * np.cos(ang[seen]), origin[1] + r[seen] * np.sin(ang[seen])
        ix, iy, ok = self.cells(hx, hy)
        np.add.at(self.hits, (iy[ok], ix[ok]), 1)
        reach = np.where(np.isnan(r), 0.0, np.minimum(r, Perception.SCAN_RANGE) - 0.1)
        m = self._t[None, :] < reach[:, None]
        fx = (origin[0] + self._t[None, :] * np.cos(ang)[:, None])[m]
        fy = (origin[1] + self._t[None, :] * np.sin(ang)[:, None])[m]
        ix, iy, ok = self.cells(fx, fy)
        once = np.zeros_like(self.free, dtype=bool)
        once[iy[ok], ix[ok]] = True
        self.free += once

    def occupied(self):
        # a real surface stops the rays; a cell they mostly pass through only caught range noise
        return (self.hits >= 2) & (self.hits > 0.3 * self.free)

    def blocked(self):
        """Planning map: cells with an obstacle in them, or that were never seen to be free
        (the far side of a parked car is unknown, not empty)."""
        seen = np.pad(self.free >= 1, 2)
        known = np.zeros((self.ny, self.nx), dtype=bool)
        for dy in range(5):                 # grow by 2 cells to close the gaps between rays
            for dx in range(5):
                known |= seen[dy:dy + self.ny, dx:dx + self.nx]
        return self.occupied() | ~known

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
    """One painted line, estimated from every detection associated with it."""
    MAX_DETS = 160

    def __init__(self, det, t):
        self.dets = [det]
        self.born = t
        self.c = np.zeros(2)
        self.d = np.array([1.0, 0.0])
        self.lo = self.hi = 0.0
        self.fit()

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
        self.fit()

    def fit(self):
        """Weighted total least squares line + extent from the coverage of the detections."""
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
        self.c, self.d, self.lo, self.hi = c, d, lo, hi


class LineMap:
    MIN_HITS = 5

    def __init__(self):
        self.tracks = []

    def confirmed(self):
        return [t for t in self.tracks if t.hits >= self.MIN_HITS and t.length > 1.2]

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
                if abs(e[0] * trk.d[1] - e[1] * trk.d[0]) > 0.21:
                    continue
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
                self.tracks.append(LineTrack(det, t))
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
                        a.fit()
                        del self.tracks[j]
                        continue
                j += 1
            i += 1


# =============================================================================
# Stall inference: a stall is the space between two neighbouring parallel lines
# =============================================================================

class Slot:
    FREE, OCCUPIED, UNKNOWN = "free", "occupied", "unknown"

    def __init__(self, kind, center, u_in, along, width, depth, corners):
        self.kind = kind            # 'perpendicular' | 'angled' | 'parallel'
        self.center = center        # where the middle of the parked car should be
        self.u_in = u_in            # unit vector from the lane into the stall
        self.along = along          # parallel stalls: unit vector along the kerb
        self.width = width
        self.depth = depth
        self.corners = corners      # entrance A, entrance B, back B, back A
        self.status = Slot.UNKNOWN
        self.neighbors = [False, False]
        self.hits = 0               # detections behind the weaker of the two lines
        self.region = None          # (mask, slices) of the grid cells between the two lines

    def goal(self, nose_in, travel_dir):
        """Rear-axle pose of the parked car."""
        if self.kind == "parallel":
            t = self.along if self.along @ travel_dir >= 0.0 else -self.along
            th = math.atan2(t[1], t[0])
        else:
            th = math.atan2(self.u_in[1], self.u_in[0]) + (0.0 if nose_in else math.pi)
        return (self.center[0] - EGO.center * math.cos(th),
                self.center[1] - EGO.center * math.sin(th), wrap(th))


def find_slots(tracks, trail, grid):
    """Pair up confirmed line tracks into stalls and classify them with the occupancy grid.
    trail: recent ego positions (N,2), used to tell the open end of a stall from its back."""
    slots = []
    n = len(tracks)
    trail = np.asarray(trail)
    for i in range(n):
        for j in range(i + 1, n):
            a, b = tracks[i], tracks[j]
            db = b.d if a.d @ b.d >= 0.0 else -b.d
            if abs(a.d[0] * db[1] - a.d[1] * db[0]) > 0.14:
                continue
            d = a.d * a.length + db * b.length
            d = d / np.hypot(*d)
            nv = np.array([-d[1], d[0]])
            sep = (b.c - a.c) @ nv
            sa = np.sort(np.array(a.ends()) @ d)
            sb = np.sort(np.array(b.ends()) @ d)
            overlap = min(sa[1], sb[1]) - max(sa[0], sb[0])
            if 2.2 <= abs(sep) <= 3.5 and min(a.length, b.length) >= 3.5 and overlap >= 3.0:
                parallel = False
            elif (5.0 <= abs(sep) <= 7.8 and 1.5 <= a.length <= 3.6 and 1.5 <= b.length <= 3.6
                  and overlap >= 1.2):
                # the two ticks must be neighbours: no third tick in between
                na, nb = sorted((a.c @ nv, b.c @ nv))
                if any(abs(k.d @ nv) < 0.2 and na + 1.0 < k.c @ nv < nb - 1.0 and
                       sa[0] - 2.0 < k.c @ d < sa[1] + 2.0 and k.length < 3.6
                       for k in tracks if k is not a and k is not b):
                    continue
                parallel = True
            else:
                continue

            # lateral position of each line and of the open (lane-side) end
            n_a, n_b = a.c @ nv, b.c @ nv
            n_mid = 0.5 * (n_a + n_b)
            p_lo = d * 0.5 * (sa[0] + sb[0]) + nv * n_mid
            p_hi = d * 0.5 * (sa[1] + sb[1]) + nv * n_mid
            d_lo = np.hypot(*(trail - p_lo).T).min()
            d_hi = np.hypot(*(trail - p_hi).T).min()
            if d_lo <= d_hi:
                u_in, in_a, in_b, bk_a, bk_b = d, sa[0], sb[0], sa[1], sb[1]
            else:
                u_in, in_a, in_b, bk_a, bk_b = -d, -sa[1], -sb[1], -sa[0], -sb[0]
            nu = np.array([-u_in[1], u_in[0]])
            la, lb = a.c @ nu, b.c @ nu
            s0, s1 = max(in_a, in_b), min(bk_a, bk_b)
            corners = np.array([u_in * in_a + nu * la, u_in * in_b + nu * lb,
                                u_in * bk_b + nu * lb, u_in * bk_a + nu * la])
            if parallel:
                s_c = 0.5 * (in_a + in_b) + min(max(0.5 * (s1 - s0), 1.2), 1.4)
                slot = Slot("parallel", u_in * s_c + nu * 0.5 * (la + lb), u_in, nu,
                            abs(sep), s1 - s0, corners)
            else:
                s_c = s0 + min(max(0.5 * (s1 - s0), 2.65), 2.95)
                skew = abs(in_a - in_b) / max(abs(la - lb), 1e-6)
                slot = Slot("perpendicular" if skew < 0.2 else "angled",
                            u_in * s_c + nu * 0.5 * (la + lb), u_in, nu, abs(sep), s1 - s0, corners)
            slot.hits = min(a.hits, b.hits)
            _classify(slot, grid, s0, s1, min(la, lb), max(la, lb))
            if parallel:
                _align_with_kerb(slot, grid, s1, min(la, lb), max(la, lb))
            slots.append(slot)
    return slots


def _align_with_kerb(slot, grid, s1, l0, l1):
    """Parallel stalls: the two short ticks give a poor heading. If the range scan shows the
    kerb behind the stall, park parallel to it and 0.3 m off it, like a driver would."""
    c = slot.corners
    win = grid.window(c[:, 0].min() - 1.5, c[:, 1].min() - 1.5, c[:, 0].max() + 1.5, c[:, 1].max() + 1.5)
    if win is None:
        return
    X, Y, sl = win
    u, nu = slot.u_in, slot.along
    S, Lc = X * u[0] + Y * u[1], X * nu[0] + Y * nu[1]
    m = grid.occupied()[sl] & (S > s1 - 0.6) & (S < s1 + 0.9) & (Lc > l0 + 0.2) & (Lc < l1 - 0.2)
    if m.sum() < 25 or np.ptp(Lc[m]) < 3.5:
        return
    mid = 0.5 * (l0 + l1)
    b, a = np.polyfit(Lc[m] - mid, S[m], 1)
    keep = np.abs(S[m] - (a + b * (Lc[m] - mid))) < 0.15
    if keep.sum() < 20:
        return
    b, a = np.polyfit(Lc[m][keep] - mid, S[m][keep], 1)
    along = (nu + b * u) / math.hypot(1.0, b)
    u_k = np.array([along[1], -along[0]])
    if u_k @ u < 0.0:
        u_k = -u_k
    # the fitted line runs through the middle of the kerb's hit cells; its face is ~half a cell nearer
    slot.center = u * (a - 0.05) + nu * mid - u_k * (EGO.half_width + 0.30)
    slot.u_in, slot.along = u_k, along


def _classify(slot, grid, s0, s1, l0, l1):
    """Free / occupied / unknown from the grid cells inside the stall, plus neighbours."""
    c = slot.corners
    win = grid.window(c[:, 0].min() - 3.0, c[:, 1].min() - 3.0, c[:, 0].max() + 3.0, c[:, 1].max() + 3.0)
    if win is None:
        return
    X, Y, sl = win
    u, nu = slot.u_in, np.array([-slot.u_in[1], slot.u_in[0]])
    S = X * u[0] + Y * u[1]
    Lc = X * nu[0] + Y * nu[1]
    occ = grid.occupied()[sl]
    free = (grid.free[sl] >= 2) & ~occ
    lat = 0.35 if slot.kind != "parallel" else 0.6
    back = 0.45
    inside = (S > s0 + 0.1) & (S < s1 - back) & (Lc > l0 + lat) & (Lc < l1 - lat)
    cells = max(int(inside.sum()), 1)
    n_occ = int((occ & inside).sum())
    frac_free = (free & inside).sum() / cells
    if n_occ >= 4:
        slot.status = Slot.OCCUPIED
    elif n_occ <= 1 and frac_free >= 0.6:
        slot.status = Slot.FREE
    side = (S > s0 - 0.3) & (S < s1 - back)
    reach = 2.4 if slot.kind != "parallel" else 3.0
    slot.neighbors = [bool((occ & side & (Lc < l0 - 0.05) & (Lc > l0 - reach)).sum() >= 4),
                      bool((occ & side & (Lc > l1 + 0.05) & (Lc < l1 + reach)).sum() >= 4)]
    slot.region = (S > s0 - 1.0) & (S < s1 - 0.2) & (Lc > l0 + 0.1) & (Lc < l1 - 0.1), sl


# =============================================================================
# Reeds-Shepp curves for a unit turning radius. The same formulas run on
# scalars (math backend, used for analytic expansions) and on arrays (numpy
# backend, used once to tabulate the planner heuristic).
# =============================================================================

class _M:
    sin, cos, atan2, sqrt, hypot, asin, acos = (math.sin, math.cos, math.atan2, math.sqrt,
                                                math.hypot, math.asin, math.acos)
    fmin, fmax = min, max

    @staticmethod
    def where(c, a, b):
        return a if c else b


class _N:
    sin, cos, atan2, sqrt, hypot, asin, acos = (np.sin, np.cos, np.arctan2, np.sqrt,
                                                np.hypot, np.arcsin, np.arccos)
    fmin, fmax, where = np.minimum, np.maximum, np.where


_HP = 0.5 * math.pi
_Z = 1e-9
_SWAP = str.maketrans("LR", "RL")


def _m2pi(m, x):
    return m.atan2(m.sin(x), m.cos(x))


def _LpSpLp(m, x, y, phi):
    xi, eta = x - m.sin(phi), y - 1.0 + m.cos(phi)
    u, t = m.hypot(xi, eta), m.atan2(eta, xi)
    v = _m2pi(m, phi - t)
    return (t >= -_Z) & (v >= -_Z), t, u, v


def _LpSpRp(m, x, y, phi):
    xi, eta = x + m.sin(phi), y - 1.0 - m.cos(phi)
    u1, t1 = xi * xi + eta * eta, m.atan2(eta, xi)
    u = m.sqrt(m.fmax(u1 - 4.0, 0.0))
    t = _m2pi(m, t1 + m.atan2(2.0, u))
    v = _m2pi(m, t - phi)
    return (u1 >= 4.0) & (t >= -_Z) & (v >= -_Z), t, u, v


def _LpRmL(m, x, y, phi):
    xi, eta = x - m.sin(phi), y - 1.0 + m.cos(phi)
    u1, theta = m.hypot(xi, eta), m.atan2(eta, xi)
    u = -2.0 * m.asin(m.fmin(0.25 * u1, 1.0))
    t = _m2pi(m, theta + 0.5 * u + math.pi)
    v = _m2pi(m, phi - t + u)
    return (u1 <= 4.0) & (t >= -_Z) & (u <= _Z), t, u, v


def _tau_omega(m, u, v, xi, eta, phi):
    delta = _m2pi(m, u - v)
    A = m.sin(u) - m.sin(delta)
    B = m.cos(u) - m.cos(delta) - 1.0
    t1 = m.atan2(eta * A - xi * B, xi * A + eta * B)
    t2 = 2.0 * (m.cos(delta) - m.cos(v) - m.cos(u)) + 3.0
    tau = m.where(t2 < 0.0, _m2pi(m, t1 + math.pi), _m2pi(m, t1))
    return tau, _m2pi(m, tau - u + v - phi)


def _LpRupLumRm(m, x, y, phi):
    xi, eta = x + m.sin(phi), y - 1.0 - m.cos(phi)
    rho = 0.25 * (2.0 + m.hypot(xi, eta))
    u = m.acos(m.fmin(rho, 1.0))
    t, v = _tau_omega(m, u, -u, xi, eta, phi)
    return (rho <= 1.0) & (t >= -_Z) & (v <= _Z), t, u, v


def _LpRumLumRp(m, x, y, phi):
    xi, eta = x + m.sin(phi), y - 1.0 - m.cos(phi)
    rho = (20.0 - xi * xi - eta * eta) / 16.0
    u = -m.acos(m.fmin(m.fmax(rho, 0.0), 1.0))
    t, v = _tau_omega(m, u, u, xi, eta, phi)
    return (rho >= 0.0) & (rho <= 1.0) & (u >= -_HP) & (t >= -_Z) & (v >= -_Z), t, u, v


def _LpRmSmLm(m, x, y, phi):
    xi, eta = x - m.sin(phi), y - 1.0 + m.cos(phi)
    rho, theta = m.hypot(xi, eta), m.atan2(eta, xi)
    r = m.sqrt(m.fmax(rho * rho - 4.0, 0.0))
    u = 2.0 - r
    t = _m2pi(m, theta + m.atan2(r, -2.0))
    v = _m2pi(m, phi - _HP - t)
    return (rho >= 2.0) & (t >= -_Z) & (u <= _Z) & (v <= _Z), t, u, v


def _LpRmSmRm(m, x, y, phi):
    xi, eta = x + m.sin(phi), y - 1.0 - m.cos(phi)
    rho, t = m.hypot(xi, eta), m.atan2(xi, -eta)
    u = 2.0 - rho
    v = _m2pi(m, t + _HP - phi)
    return (rho >= 2.0) & (t >= -_Z) & (u <= _Z) & (v <= _Z), t, u, v


def _LpRmSLmRp(m, x, y, phi):
    xi, eta = x + m.sin(phi), y - 1.0 - m.cos(phi)
    rho = m.hypot(xi, eta)
    u = 4.0 - m.sqrt(m.fmax(rho * rho - 4.0, 0.0))
    t = _m2pi(m, m.atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta))
    v = _m2pi(m, t - phi)
    return (rho >= 2.0) & (u <= _Z) & (t >= -_Z) & (v >= -_Z), t, u, v


def _rs_words(m, x, y, phi):
    """Every Reeds-Shepp word from the origin to (x, y, phi): yields (word, valid, signed lengths)."""
    xb = x * m.cos(phi) + y * m.sin(phi)
    yb = x * m.sin(phi) - y * m.cos(phi)
    for f, r in ((1, 1), (-1, 1), (1, -1), (-1, -1)):     # f: time flip, r: reflection
        X, Y, P, XB, YB = f * x, r * y, f * r * phi, f * xb, r * yb
        w = (lambda s: s) if r > 0 else (lambda s: s.translate(_SWAP))
        ok, t, u, v = _LpSpLp(m, X, Y, P)
        yield w("LSL"), ok, (f * t, f * u, f * v)
        ok, t, u, v = _LpSpRp(m, X, Y, P)
        yield w("LSR"), ok, (f * t, f * u, f * v)
        ok, t, u, v = _LpRmL(m, X, Y, P)
        yield w("LRL"), ok, (f * t, f * u, f * v)
        ok, t, u, v = _LpRmL(m, XB, YB, P)
        yield w("LRL"), ok, (f * v, f * u, f * t)
        ok, t, u, v = _LpRupLumRm(m, X, Y, P)
        yield w("LRLR"), ok, (f * t, f * u, -f * u, f * v)
        ok, t, u, v = _LpRumLumRp(m, X, Y, P)
        yield w("LRLR"), ok, (f * t, f * u, f * u, f * v)
        ok, t, u, v = _LpRmSmLm(m, X, Y, P)
        yield w("LRSL"), ok, (f * t, -f * _HP, f * u, f * v)
        ok, t, u, v = _LpRmSmRm(m, X, Y, P)
        yield w("LRSR"), ok, (f * t, -f * _HP, f * u, f * v)
        ok, t, u, v = _LpRmSmLm(m, XB, YB, P)
        yield w("LSRL"), ok, (f * v, f * u, -f * _HP, f * t)
        ok, t, u, v = _LpRmSmRm(m, XB, YB, P)
        yield w("RSRL"), ok, (f * v, f * u, -f * _HP, f * t)
        ok, t, u, v = _LpRmSLmRp(m, X, Y, P)
        yield w("LRSLR"), ok, (f * t, -f * _HP, f * u, -f * _HP, f * v)


def rs_paths(x, y, phi):
    return [(w, lens) for w, ok, lens in _rs_words(_M, x, y, phi) if ok]


def rs_length_table(xs, ys, phis):
    """Shortest Reeds-Shepp length (unit radius) over a grid of relative poses."""
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    best = np.full(X.shape + (len(phis),), np.inf, dtype=np.float32)
    for k, phi in enumerate(phis):          # one heading at a time keeps the working set small
        P = np.full(X.shape, float(phi))
        for _, ok, lens in _rs_words(_N, X, Y, P):
            total = sum(np.abs(l) for l in lens)
            best[:, :, k] = np.where(ok & (total < best[:, :, k]), total, best[:, :, k])
    return best


def rs_sample(start, word, lens, radius, ds):
    """Integrate a word from a start pose. Rows are (x, y, heading, direction, curvature)."""
    x, y, th = start
    rows = []
    for c, l in zip(word, lens):
        L = abs(l) * radius
        if L < 1e-4:
            continue
        d = 1.0 if l > 0 else -1.0
        n = max(1, int(math.ceil(L / ds)))
        s = np.linspace(L / n, L, n) * d
        if c == "S":
            k = 0.0
            px, py, pth = x + s * math.cos(th), y + s * math.sin(th), np.full(n, th)
        else:
            k = (1.0 if c == "L" else -1.0) / radius
            pth = th + k * s
            px = x + (np.sin(pth) - math.sin(th)) / k
            py = y - (np.cos(pth) - math.cos(th)) / k
        rows.append(np.stack([px, py, pth, np.full(n, d), np.full(n, k)], axis=1))
        x, y, th = px[-1], py[-1], pth[-1]
    return np.concatenate(rows) if rows else np.zeros((0, 5))


# =============================================================================
# Configuration space: for every heading bin, the set of rear-axle positions at
# which the (inflated) footprint touches an occupied cell. One FFT correlation
# of the occupancy grid with the rotated footprint per heading.
# =============================================================================

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
        self.v_ref = np.minimum(v_max, np.clip(STEER_RATE * EGO.kappa / np.maximum(dk, 1e-6), 0.5, v_max))

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


# =============================================================================
# Path tracking: model predictive control of the steering, with the steering
# gain of the car identified online, and a PI loop on speed
# =============================================================================

def nnls(E, f, max_iter=200):
    """Lawson-Hanson active-set method: minimise ||E x - f|| subject to x >= 0.
    Returns x and the number of iterations."""
    n = E.shape[1]
    P = np.zeros(n, dtype=bool)                 # the passive set: variables allowed to be positive
    x = np.zeros(n)
    w = E.T @ f
    tol = 1e-10 * E.shape[0] * max(np.abs(E).max() * np.abs(f).max(), 1e-30)
    for it in range(max_iter):
        cand = np.where(P, -np.inf, w)
        j = int(np.argmax(cand))
        if cand[j] <= tol:
            break
        P[j] = True
        while True:
            s = np.zeros(n)
            s[P] = np.linalg.lstsq(E[:, P], f, rcond=None)[0]
            if s[P].min() > 0.0:
                break
            neg = P & (s <= 0.0)                # step towards s until the first variable hits zero
            x = x + np.min(x[neg] / (x[neg] - s[neg])) * (s - x)
            P &= x > 1e-14
            if not P.any():
                s = np.zeros(n)
                break
        x = s
        w = E.T @ (f - E @ x)
    return x, it + 1


class LateralMPC:
    """Linear MPC of the lateral motion along a path, written in travelled distance instead of
    time so that it stays well posed down to walking pace.

    State x = (e, psi): lateral and heading error of the rear axle w.r.t. the path. Over one
    step of length h, driving in direction d = +-1 with curvature kappa where the path has
    curvature kappa_ref, the kinematic bicycle gives (to first order in the errors)

        e'   = e + d h psi + h^2/2 (kappa - kappa_ref)
        psi' = psi + d h (kappa - kappa_ref)

    The decision variables are the N curvatures over the horizon. The cost penalises the
    errors, the deviation from the path curvature and curvature changes; the constraints are
    the steering limit |kappa| <= kappa_max and the steering rate. The Hessian of the condensed
    QP is constant, so it is factorised once; each solve turns the QP into a least-distance
    problem and solves that exactly with a non-negative least squares active-set method."""
    N = 20                   # horizon steps
    H = 0.2                  # step [m]  -> 4 m look-ahead
    Q_E, Q_PSI = 10.0, 6.0   # error weights
    Q_END = 3.0              # extra weight on the last step
    R_K = 1.0                # weight on (kappa - kappa_ref)
    R_DK = 1.0               # weight on curvature changes

    def __init__(self):
        N, h = self.N, self.H
        self.D = np.eye(N) - np.eye(N, k=-1)                 # (D K)_k = K_k - K_{k-1}
        Ac = np.vstack([np.eye(N), self.D])                  # rows: curvature, then curvature change
        G = np.vstack([Ac, -Ac])                             # G K <= [hi; -lo]
        self.qp = {}
        for d in (1, -1):
            A = np.array([[1.0, d * h], [0.0, 1.0]])
            B = np.array([0.5 * h * h, d * h])
            Phi, Gam = np.zeros((2 * N, 2)), np.zeros((2 * N, N))
            Ak = np.eye(2)
            for k in range(N):
                Ak = A @ Ak
                Phi[2 * k:2 * k + 2] = Ak
                for j in range(k + 1):
                    Gam[2 * k:2 * k + 2, j] = np.linalg.matrix_power(A, k - j) @ B
            w = np.tile([float(self.Q_E), float(self.Q_PSI)], N)
            w[-2:] *= self.Q_END
            GQ = Gam.T * w
            Hm = 2.0 * (GQ @ Gam + self.R_K * np.eye(N) + self.R_DK * self.D.T @ self.D)
            Linv = np.linalg.inv(np.linalg.cholesky(Hm))      # Hm = L L^T
            self.qp[d] = dict(Phi=Phi, Gam=Gam, GQ=GQ, Linv=Linv, Gt=-G @ Linv.T, GHinv=G @ Linv.T @ Linv)
        self.iters = 0

    def solve(self, d, e0, psi0, k_prev, k_ref, k_max, dk_max):
        """Returns the curvature sequence and the predicted (e, psi) over the horizon."""
        q, N = self.qp[d], self.N
        x0 = np.array([e0, psi0])
        c = np.zeros(N)
        c[0] = k_prev
        f = 2.0 * (q["GQ"] @ (q["Phi"] @ x0 - q["Gam"] @ k_ref) - self.R_K * k_ref - self.R_DK * self.D.T @ c)
        hi = np.concatenate([np.full(N, k_max), c + dk_max])
        lo = np.concatenate([np.full(N, -k_max), c - dk_max])
        # With y = L^T K + L^-1 f the QP is "minimise |y| subject to Gt y >= ht", which Lawson and
        # Hanson reduce to one non-negative least squares problem
        ht = -(np.concatenate([hi, -lo]) + q["GHinv"] @ f)
        E = np.vstack([q["Gt"].T, ht[None, :]])
        rhs = np.zeros(N + 1)
        rhs[-1] = 1.0
        u, self.iters = nnls(E, rhs)
        r = E @ u - rhs
        if abs(r[-1]) > 1e-12:
            K = q["Linv"].T @ (-r[:-1] / r[-1] - q["Linv"] @ f)
        else:                 # only if the constraints contradict each other: fall back to clipping
            K = -q["Linv"].T @ (q["Linv"] @ f)
        K = np.clip(K, -k_max, k_max)
        pred = (q["Phi"] @ x0 + q["Gam"] @ (K - k_ref)).reshape(N, 2)
        return K, pred


class SteeringGain:
    """Online estimate of the car's steering gain g in  curvature = g * steering input,
    separately for forward and reverse, by recursive least squares on the measured curvature
    (yaw rate / speed). It starts from the kinematic value of the Chrono model, so no
    calibration data is needed."""
    FORGET = 0.995

    def __init__(self):
        self.g = {1: EGO.kappa, -1: EGO.kappa}
        self.P = {1: 0.5, -1: 0.5}
        self.prev = 0.0

    def update(self, d, steer, v, yaw_rate, dt):
        settled = abs(steer - self.prev) < 0.5 * dt          # the yaw response lags a moving wheel
        self.prev = steer
        if abs(v) < 0.5 or abs(steer) < 0.15 or not settled:
            return
        P = self.P[d]
        gain = P * steer / (self.FORGET + steer * P * steer)
        g = self.g[d] + gain * (yaw_rate / v - self.g[d] * steer)
        self.g[d] = min(max(g, 0.4 * EGO.kappa), 2.5 * EGO.kappa)
        self.P[d] = min((P - gain * steer * P) / self.FORGET, 0.5)


class MpcTracker:
    """Follows one path segment at a time: turn the wheels while standing, drive, stop."""
    A_DEC = 0.5          # deceleration used to approach the end of a segment [m/s^2]
    A_ACC = 0.7
    V_CREEP = 0.15

    def __init__(self):
        self.seg = None
        self.steer = 0.0
        self.phase = "idle"
        self.done = True
        self.err = (0.0, 0.0, 0.0)       # lateral, heading, remaining distance
        self.mpc = LateralMPC()
        self.gain = SteeringGain()
        self.horizon = np.zeros((0, 2))  # predicted rear-axle positions, for display
        self.k_plan = np.zeros(0)        # planned curvatures over the horizon
        self.k_ref = np.zeros(0)

    def start(self, seg, t, presteer=True):
        self.seg, self.i, self.done = seg, 0, False
        self.phase = "steer" if presteer else "go"
        self.t_phase, self.integ, self.v_cmd, self.t_still = t, 0.0, 0.0, None

    def stop(self):
        if self.phase in ("steer", "go"):
            self.phase, self.t_still = "stop", None

    def update(self, pose, v, yaw_rate, t, dt):
        seg = self.seg
        x, y, th = pose
        i1 = min(len(seg.x), self.i + 40)
        self.i += int(np.argmin((seg.x[self.i:i1] - x) ** 2 + (seg.y[self.i:i1] - y) ** 2))
        i, d = self.i, seg.dir
        e = -math.sin(seg.th[i]) * (x - seg.x[i]) + math.cos(seg.th[i]) * (y - seg.y[i])
        psi = wrap(th - seg.th[i])
        s_rem = seg.length - seg.s[i]
        if s_rem < 1.5:        # near the end, measure what is left along the final heading
            s_rem = d * ((seg.x[-1] - x) * math.cos(seg.th[-1]) + (seg.y[-1] - y) * math.sin(seg.th[-1]))
        self.err = (e, psi, s_rem)

        # steering: MPC over the next few metres of path, with the current estimate of the gain
        self.gain.update(d, self.steer, v, yaw_rate, dt)
        g = self.gain.g[d]
        mpc = self.mpc
        ahead = seg.s[i] + (np.arange(mpc.N) + 0.5) * mpc.H
        k_ref = np.interp(ahead, seg.s, seg.kappa)           # holds the last curvature past the end
        dk_max = min(STEER_RATE * g * mpc.H / max(abs(v), 0.3), 2.0 * g)
        K, pred = mpc.solve(d, e, psi, g * self.steer, k_ref, g, dk_max)
        target = min(max(K[0] / g, -1.0), 1.0)
        self.steer += max(-STEER_RATE * dt, min(STEER_RATE * dt, target - self.steer))
        j = np.minimum(np.searchsorted(seg.s, seg.s[i] + (np.arange(mpc.N) + 1.0) * mpc.H), len(seg.x) - 1)
        self.horizon = np.stack([seg.x[j] - pred[:, 0] * np.sin(seg.th[j]),
                                 seg.y[j] + pred[:, 0] * np.cos(seg.th[j])], axis=1)
        self.k_plan, self.k_ref = K, k_ref

        va = v * d
        if self.phase == "steer":      # stationary: turn the wheels before moving off
            if (abs(target - self.steer) < 0.03 and t - self.t_phase > 0.3) or t - self.t_phase > 2.5:
                self.phase = "go"
            return self.steer, 0.0, 0.6
        if self.phase == "go":
            if s_rem <= 0.015 + 0.06 * max(va, 0.0):
                self.phase, self.t_still = "stop", None
            else:
                v_tgt = max(min(seg.v_ref[i], math.sqrt(2.0 * self.A_DEC * max(s_rem, 0.0))), self.V_CREEP)
                self.v_cmd = min(v_tgt, self.v_cmd + self.A_ACC * dt)
                err = self.v_cmd - va
                self.integ = min(max(self.integ + 0.5 * err * dt, 0.0), 0.5)
                throttle = min(max(0.5 * err + self.integ, 0.0), 0.7)
                brake = min(max(1.5 * (-err - 0.08), 0.0), 0.8)
                if brake > 0.0:
                    throttle, self.integ = 0.0, 0.8 * self.integ
                return self.steer, throttle, brake
        # stop: hold the brake until the car has been still for a moment
        if abs(v) < 0.03:
            if self.t_still is None:
                self.t_still = t
            elif t - self.t_still > 0.3:
                self.done, self.phase = True, "idle"
        else:
            self.t_still = None
        return self.steer, 0.0, 0.6


# =============================================================================
# Chrono world: sedan, flat terrain, painted lines, kerbs and parked cars
# =============================================================================

class World:
    def __init__(self, scn, visual=True, tire="tmeasy"):
        self.scn = scn
        x, y, th = scn.start
        sedan = veh.Sedan()
        sedan.SetContactMethod(chrono.ChContactMethod_SMC)
        sedan.SetChassisCollisionType(veh.CollisionType_HULLS)    # so that hitting a parked car is physical
        sedan.SetChassisFixed(False)
        sedan.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(x, y, 0.25), chrono.QuatFromAngleZ(th)))
        sedan.SetTireType(veh.TireModelType_PAC02 if tire == "pac02" else veh.TireModelType_TMEASY)
        sedan.SetTireStepSize(1e-3)
        sedan.SetBrakeType(veh.BrakeType_SHAFTS)     # the simple brake cannot hold the car still
        sedan.Initialize()
        vt = chrono.VisualizationType_MESH if visual else chrono.VisualizationType_NONE
        sedan.SetChassisVisualizationType(vt)
        sedan.SetSuspensionVisualizationType(chrono.VisualizationType_NONE)
        sedan.SetSteeringVisualizationType(chrono.VisualizationType_NONE)
        sedan.SetWheelVisualizationType(vt)
        sedan.SetTireVisualizationType(vt)
        self.sedan = sedan
        self.car = sedan.GetVehicle()
        self.system = sedan.GetSystem()
        EGO.read(self.car)
        self.system.SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)
        self.gearbox = self.car.GetTransmission().asAutomatic()
        self.inputs = veh.DriverInputs()

        mat = chrono.ChContactMaterialSMC()
        mat.SetFriction(0.9)
        mat.SetRestitution(0.01)
        self.terrain = veh.RigidTerrain(self.system)
        x0, y0, x1, y1 = scn.bounds
        patch = self.terrain.AddPatch(mat, chrono.ChCoordsysd(
            chrono.ChVector3d(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.0), chrono.QUNIT),
            (x1 - x0) + 60.0, (y1 - y0) + 60.0)
        patch.SetColor(chrono.ChColor(0.23, 0.23, 0.25))
        self.terrain.Initialize()

        self.contact_mat = mat
        self._decor(visual)
        self._parked_cars(visual)

    def _box(self, body, lx, ly, lz, pos, yaw, rgb):
        shape = chrono.ChVisualShapeBox(lx, ly, lz)
        shape.SetColor(chrono.ChColor(*rgb))
        body.AddVisualShape(shape, chrono.ChFramed(chrono.ChVector3d(*pos), chrono.QuatFromAngleZ(yaw)))

    def _decor(self, visual):
        if not visual:
            return
        body = chrono.ChBody()
        body.SetFixed(True)
        for x1, y1, x2, y2, color in self.scn.lines:
            rgb = (0.95, 0.95, 0.95) if color == "white" else (0.95, 0.78, 0.10)
            self._box(body, math.hypot(x2 - x1, y2 - y1), 0.12, 0.01,
                      (0.5 * (x1 + x2), 0.5 * (y1 + y2), 0.006), math.atan2(y2 - y1, x2 - x1), rgb)
        for cx, cy, lx, ly in self.scn.curbs:
            self._box(body, lx, ly, 0.15, (cx, cy, 0.075), 0.0, (0.66, 0.66, 0.64))
        for cx, cy, lx, ly, rgb in self.scn.pads:
            self._box(body, lx, ly, 0.14, (cx, cy, 0.07), 0.0, rgb)
        self.system.Add(body)

    def _parked_cars(self, visual):
        data = chrono.GetChronoDataPath() + "vehicle/"
        for car in self.scn.cars:
            m = parked_model(car["model"])
            half = 0.5 * (m["x1"] - m["x0"])
            mid = 0.5 * (m["x1"] + m["x0"])
            c, s = math.cos(car["yaw"]), math.sin(car["yaw"])
            body = chrono.ChBody()
            body.SetFixed(True)
            body.SetPos(chrono.ChVector3d(car["cx"] - mid * c, car["cy"] - mid * s, m["z"]))
            body.SetRot(chrono.QuatFromAngleZ(car["yaw"]))
            shape = chrono.ChCollisionShapeBox(self.contact_mat, 2.0 * half, 2.0 * m["hw"], 1.2)
            body.AddCollisionShape(shape, chrono.ChFramed(chrono.ChVector3d(mid, 0.0, 0.5), chrono.QUNIT))
            body.EnableCollision(True)
            if visual:
                mesh = data + m["mesh"] % car["color"]
                if os.path.exists(mesh):
                    parts = [(mesh, (0.0, 0.0, 0.0), 0.0)]
                    for wx, wy, wz in m["wheels"]:
                        for sgn in (1.0, -1.0):
                            for f in (m["rim"], m["tire"]):
                                parts.append((data + f, (wx, sgn * wy, wz), 0.0 if sgn > 0 else math.pi))
                    for f, pos, yaw in parts:
                        mesh_obj = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(f, True, True)
                        shape = chrono.ChVisualShapeTriangleMesh(mesh_obj, True)     # keeps the .mtl colours
                        body.AddVisualShape(shape, chrono.ChFramed(chrono.ChVector3d(*pos),
                                                                   chrono.QuatFromAngleZ(yaw)))
                else:
                    self._box(body, 2.0 * half, 2.0 * m["hw"], 1.3, (mid, 0.0, 0.5), 0.0, (0.5, 0.5, 0.55))
            self.system.Add(body)

    def state(self):
        """Rear-axle pose (x, y, heading) and signed forward speed."""
        frame = self.car.GetChassisBody().GetFrameRefToAbs()
        p = frame.GetPos()
        ax = frame.GetRotMat().GetAxisX()
        th = math.atan2(ax.y, ax.x)
        return (p.x - EGO.ref_to_rear * math.cos(th), p.y - EGO.ref_to_rear * math.sin(th), th), self.car.GetSpeed()

    def yaw_rate(self):
        return self.car.GetYawRate()

    def set_direction(self, d):
        AT = veh.ChAutomaticTransmission
        self.gearbox.SetDriveMode(AT.DriveMode_FORWARD if d > 0 else AT.DriveMode_REVERSE)

    def step(self, steer, throttle, brake):
        t = self.system.GetChTime()
        self.inputs.m_steering, self.inputs.m_throttle, self.inputs.m_braking = steer, throttle, brake
        self.terrain.Synchronize(t)
        self.sedan.Synchronize(t, self.inputs, self.terrain)
        self.terrain.Advance(STEP)
        self.sedan.Advance(STEP)


# =============================================================================
# The parking agent: perception -> map -> stall decision -> plan -> track
# =============================================================================

class ParkingSim:
    MAX_REPLANS = 6
    MAX_CORRECTIONS = 2

    def __init__(self, args):
        self.args = args
        self.scn = make_scenario(args.type, args.cars, args.side, args.angle, args.seed)
        rng = np.random.default_rng(args.seed + 7919)
        self.world = World(self.scn, visual=not args.headless, tire=args.tire)
        self.sensor = Perception(self.scn, args.noise, rng)
        self.grid = GridMap(self.scn.bounds)
        self.lines = LineMap()
        self.planner = Planner()
        self.planner.table()          # tabulate the Reeds-Shepp heuristic up front (~1 s)
        self.tracker = MpcTracker()
        self.obstacles = self.scn.obstacle_polys()

        self.state = "SETTLE"
        self.message = ""
        self.slots, self.target, self.goal = [], None, None
        self.path, self.seg_i = [], 0
        self.rejected = []
        self.watch = []                # recent estimates of the leading candidate stall
        self.manual = None             # user-requested pose of the car's middle (x, y, heading)
        self.nose_in = False
        self.trail = [np.array(self.scn.start[:2])]
        self.dets, self.scan = [], np.zeros((0, 2))
        self.cmd = (0.0, 0.0, 1.0)
        self.steps = 0
        self.t_still = None
        self.replans = self.corrections = self.blocked = 0
        self.must_replan = False
        self.plan_info = None          # statistics and search tree of the last plan
        self.plan_thread = self.plan_result = None
        self.plan_time = 0.0
        self.min_clearance = float("inf")
        self.gear_changes = 0
        self.result = None
        self.pose, self.speed = self.world.state()
        self.travel_dir = np.array([math.cos(self.pose[2]), math.sin(self.pose[2])])
        self.park_dir = self.travel_dir          # which way a parallel-parked car should face
        car = self.world.car
        print("[parking] vehicle: Chrono::Vehicle Sedan, full multibody model (%s front / %s rear suspension, "
              "%s steering, %s driveline, %s)" % (
                  car.GetSuspension(0).GetTemplateName(), car.GetSuspension(1).GetTemplateName(),
                  car.GetSteering(0).GetTemplateName(), car.GetDriveline().GetTemplateName(),
                  car.GetTire(0, veh.LEFT).GetTemplateName()), flush=True)

    @property
    def time(self):
        return self.world.system.GetChTime()

    def say(self, text):
        self.message = text
        print("[%6.2f s] %s" % (self.time, text), flush=True)

    # ---- simulation loop -----------------------------------------------------

    def advance(self, n_steps):
        """Advance the simulation; returns False while the planner is busy (time is frozen)."""
        if self.state == "PLAN":
            if self.plan_thread.is_alive():
                return False
            self._plan_done()
        for _ in range(n_steps):
            if self.state == "PLAN":
                break
            if self.steps % int(round(PERCEPTION_DT / STEP)) == 0:
                self.pose, self.speed = self.world.state()
                self._perceive()
            if self.steps % int(round(CONTROL_DT / STEP)) == 0:
                self.pose, self.speed = self.world.state()
                self._control()
            self.world.step(*self.cmd)
            self.steps += 1
        return True

    def _follow(self, seg, presteer=True):
        self.world.set_direction(seg.dir)
        self.tracker.start(seg, self.time, presteer)

    def _search_route(self):
        x, y, th = self.pose
        y0, n = self.scn.start[1], max(2, int((self.scn.route_end - x) / 0.1))
        xs = np.linspace(x, self.scn.route_end, n)
        return Segment(np.stack([xs, np.full(n, y0), np.zeros(n)], axis=1), np.zeros(n), 1, V_SEARCH)

    def _control(self):
        t, v = self.time, self.speed
        if self.state == "SETTLE":
            self.cmd = (0.0, 0.0, 1.0)
            if t > 1.0 and self.args.target == "drag":
                self.state = "WAIT"
                self.say("drag the box onto a spot and press GO")
            elif t > 1.0 and self.args.target is not None:
                self.go_to(self.args.target)
            elif t > 1.0:
                self.state = "SEARCH"
                self._follow(self._search_route(), presteer=False)
                self.say("searching for a free stall (%s, cars: %s)" % (self.scn.name, self.args.cars))
        elif self.state in ("SEARCH", "DRIVE"):
            self.cmd = self.tracker.update(self.pose, v, self.world.yaw_rate(), t, CONTROL_DT)
            if self.tracker.done:
                if self.state == "SEARCH" and self.manual is not None:
                    self.state, self.t_still = "BRAKE", None
                elif self.state == "SEARCH":
                    self._finish(False, "reached the end of the route without finding a usable stall")
                else:
                    self._segment_done()
        elif self.state == "BRAKE":
            self.cmd = (self.tracker.steer, 0.0, 0.45)
            if abs(v) < 0.03:
                if self.t_still is None:
                    self.t_still = t
                elif t - self.t_still > 0.3:
                    self._request_plan()
            else:
                self.t_still = None
        else:   # WAIT / PARKED / FAILED: straighten the wheels and hold the brake
            s = self.tracker.steer
            self.tracker.steer = s - max(-0.02, min(0.02, s))
            self.cmd = (self.tracker.steer, 0.0, 1.0)

    def _perceive(self):
        origin, ang, r, dets = self.sensor.sense(self.pose)
        self.grid.update(origin, ang, r)
        self.lines.update(dets, self.time)
        ok = np.isfinite(r)
        self.scan = np.stack([origin[0] + r[ok] * np.cos(ang[ok]), origin[1] + r[ok] * np.sin(ang[ok])], axis=1)
        self.dets = dets
        p = np.array(self.pose[:2])
        if np.hypot(*(p - self.trail[-1])) > 0.5:
            self.trail.append(p)
        self.slots = find_slots(self.lines.confirmed(), self.trail, self.grid)

        ego = ego_poly(self.pose)
        ctr = ego.mean(axis=0)
        for poly in self.obstacles:
            if np.hypot(*(poly.mean(axis=0) - ctr)) < 0.5 * np.hypot(*(poly[2] - poly[0])) + 6.0:
                self.min_clearance = min(self.min_clearance, poly_distance(ego, poly))

        if self.state == "SEARCH":
            if self.manual is None:
                self._decide()
            else:
                self._approach()
        elif self.state == "BRAKE":
            if self.target is not None:
                self._retarget()
            elif self.manual is not None:
                self._snap()
        elif self.state == "DRIVE":
            if self.target is not None:
                self._refine()
            self._monitor()

    # ---- decision ------------------------------------------------------------

    def _decide(self):
        """Pick a stall: confirmed free, close enough to be well observed, best score."""
        fwd = self.travel_dir
        left = np.array([-fwd[1], fwd[0]])
        ctr = np.array(self.pose[:2]) + EGO.center * fwd
        best = None
        for s in self.slots:
            if s.status != Slot.FREE or s.hits < 8:
                continue
            # a stall the planner turned down is left alone until the car has moved on a few metres
            # (its estimate may have been poor), and for good after three attempts
            if any(np.hypot(*(s.center - c)) < 1.5 and (n >= 3 or np.hypot(*(ctr - at)) < 3.0)
                   for c, at, n in self.rejected):
                continue
            rel = s.center - ctr
            ahead, lat = rel @ fwd, rel @ left
            if not -8.0 < ahead < 10.0:
                continue
            score = abs(ahead) + 0.3 * abs(lat) + 1.5 * sum(s.neighbors)
            if best is None or score < best[0]:
                best = (score, s)
        if best is None:
            self.watch = []
            return
        s = best[1]
        axis = math.atan2(s.u_in[1], s.u_in[0])
        if self.watch and np.hypot(*(s.center - self.watch[-1][1])) > 1.2:
            self.watch = []                    # a different stall took the lead
        self.watch = [w for w in self.watch if self.time - w[0] < 0.75] + [(self.time, s.center, axis)]
        if (self.time - self.watch[0][0] < 0.55 or
                max(np.hypot(*(w[1] - s.center)) for w in self.watch) > 0.12 or
                max(abs(wrap(w[2] - axis)) for w in self.watch) > math.radians(1.2)):
            return                             # wait until its estimate has settled
        self.target, self.park_dir = s, fwd
        if self.args.park == "auto":
            self.nose_in = s.kind == "angled" and float(s.u_in @ fwd) > 0.17
        else:
            self.nose_in = self.args.park == "forward"
        nb = {(False, False): "no neighbours", (True, True): "cars on both sides"}.get(
            tuple(s.neighbors), "a car on one side")
        how = "parallel park" if s.kind == "parallel" else ("drive in forwards" if self.nose_in else "back in")
        self.say("chose a free %s stall %.1f m ahead with %s -> %s" %
                 (s.kind, (s.center - ctr) @ fwd, nb, how))
        self.tracker.stop()
        self.state, self.t_still = "BRAKE", None

    # ---- manual target ---------------------------------------------------------

    def go_to(self, box):
        """Park with the middle of the car at box = (x, y, heading), instead of choosing a stall.
        If the box sits on a stall the map knows about, that stall becomes the target."""
        self.manual = (box[0], box[1], wrap(box[2]))
        self.target, self.path, self.seg_i, self.result = None, [], 0, None
        self.replans = self.corrections = 0
        self.must_replan = False
        self.rejected = []
        self._snap()
        x, y, th = self.pose
        on_route = abs(y - self.scn.start[1]) < 0.6 and abs(wrap(th - self.scn.start[2])) < 0.2
        ahead = (np.array(box[:2]) - np.array([x, y])) @ self.travel_dir
        if on_route and ahead > 12.0 and self.scn.route_end - x > 3.0:
            self.say("driving up to the requested spot")
            self.state = "SEARCH"
            self._follow(self._search_route(), presteer=abs(self.speed) < 0.1)
        else:
            self.tracker.stop()
            self.state, self.t_still = "BRAKE", None

    def _snap(self):
        """Adopt the detected stall under the user's box, if there is one."""
        if self.args.no_snap:
            return False
        x, y, th = self.manual
        h = np.array([math.cos(th), math.sin(th)])
        for s in self.slots:
            axis = s.along if s.kind == "parallel" else s.u_in
            if np.hypot(s.center[0] - x, s.center[1] - y) < 1.2 and abs(axis @ h) > 0.9:
                self.target = s
                self.nose_in = float(s.u_in @ h) > 0.0
                self.park_dir = h
                self.say("the box is on a %s %s stall, using its lines" % (s.status, s.kind))
                return True
        return False

    def _box_cells(self, grow=0.0):
        """Grid cells under the user's box: (mask, slices), or None if it is off the map."""
        x, y, th = self.manual
        win = self.grid.window(x - 3.5, y - 3.5, x + 3.5, y + 3.5)
        if win is None:
            return None
        X, Y, sl = win
        c, s = math.cos(th), math.sin(th)
        inside = ((np.abs((X - x) * c + (Y - y) * s) < 0.5 * EGO.length + grow) &
                  (np.abs(-(X - x) * s + (Y - y) * c) < EGO.half_width + grow))
        return inside, sl

    def _approach(self):
        """Manual target further down the lane: cruise until it is close and has been looked at."""
        if self.target is None:
            self._snap()
        if self.target is not None:
            ref, near, seen = self.target.center, 7.0, self.target.status != Slot.UNKNOWN
        else:      # no stall recognised under the box (yet): go right up to it before planning
            ref, near, cells = np.array(self.manual[:2]), 1.5, self._box_cells()
            seen = cells is None or (self.grid.free[cells[1]] >= 1)[cells[0]].mean() > 0.6 or \
                self.grid.occupied()[cells[1]][cells[0]].sum() >= 6
        ahead = (ref - np.array(self.pose[:2])) @ self.travel_dir
        if (ahead < near and seen) or ahead < -1.5:
            self.tracker.stop()
            self.state, self.t_still = "BRAKE", None

    # ---- planning --------------------------------------------------------------

    def _goal_spec(self):
        """What the planner should aim for: the chosen stall, or the pose the user asked for."""
        if self.target is not None:
            s = self.target
            nominal = s.goal(self.nose_in, self.park_dir)
            if s.kind == "parallel":     # reverse in, then pull forward to the middle of the stall
                return dict(kind=s.kind, nominal=nominal, u=s.u_in, signs=(1.0,), runs=(1.6, 1.1, 0.6, 0.0),
                            trials=sorted((abs(a) + 2.0 * b, a, b) for a in np.arange(-0.6, 0.61, 0.1)
                                          for b in np.arange(0.0, 0.31, 0.05)),
                            margins=((0.12, 0.22), (0.08, 0.15), (0.06, 0.12)))
            return dict(kind=s.kind, nominal=nominal, u=s.u_in, signs=(1.0 if self.nose_in else -1.0,),
                        runs=(3.5, 2.5, 1.5, 0.8),
                        trials=sorted((abs(a) + b, a, b) for a in np.arange(-0.3, 0.31, 0.05)
                                      for b in np.arange(0.0, 0.61, 0.15)),
                        margins=((0.25, 0.30), (0.15, 0.20), (0.08, 0.12)))
        x, y, th = self.manual
        nominal = (x - EGO.center * math.cos(th), y - EGO.center * math.sin(th), th)
        return dict(kind="manual", nominal=nominal, u=np.array([math.cos(th), math.sin(th)]),
                    signs=(1.0, -1.0), runs=(3.0, 2.0, 1.2, 0.6, 0.0),
                    trials=sorted((abs(a) + abs(b), a, b) for a in np.arange(-0.2, 0.21, 0.1)
                                  for b in np.arange(-0.2, 0.21, 0.1)),
                    margins=((0.25, 0.30), (0.15, 0.20), (0.08, 0.12)))

    def _request_plan(self):
        occ = self.grid.blocked()
        region = self.target.region if self.target is not None else self._box_cells(0.3)
        if region is not None:       # the chosen spot was judged free (by the map, or by the user)
            mask, sl = region
            occ[sl] &= ~mask | self.grid.occupied()[sl]
        self.state = "PLAN"
        self.say("planning ...")
        self.plan_result = None
        self._plan_wall = time.time()
        job = (self.pose, self._goal_spec(), occ)
        self.plan_thread = threading.Thread(target=self._plan_job, args=job, daemon=True)
        self.plan_thread.start()
        if self.args.headless:
            self.plan_thread.join()

    def _plan_job(self, start, spec, occ):
        try:
            for k, (m_lat, m_lon) in enumerate(spec["margins"]):
                self.planner.max_iter = 30000 if k == len(spec["margins"]) - 1 else 12000
                res = self._plan_once(start, spec, occ, m_lat, m_lon)
                if res is not None:
                    self.plan_result = res
                    return
        except Exception:             # never leave the main loop waiting on a dead thread
            import traceback
            traceback.print_exc()

    def _plan_once(self, start, spec, occ, m_lat, m_lon):
        g, res = self.grid, self.grid.RES
        nominal = spec["nominal"]
        win = 13.0
        i0 = max(0, int((min(start[0], nominal[0]) - win - g.x0) / res))
        i1 = min(g.nx, int((max(start[0], nominal[0]) + win - g.x0) / res))
        j0 = max(0, int((min(start[1], nominal[1]) - win - g.y0) / res))
        j1 = min(g.ny, int((max(start[1], nominal[1]) + win - g.y0) / res))
        sub = occ[j0:j1, i0:i1]
        wx0, wy0 = g.x0 + i0 * res, g.y0 + j0 * res
        cs = CSpace(sub, wx0, wy0, res, m_lat, m_lon)
        if cs.query(*start) == CSpace.HARD:
            return None
        edge = sub.copy()                      # only the rim of the blocked regions matters
        edge[1:-1, 1:-1] &= ~(sub[:-2, 1:-1] & sub[2:, 1:-1] & sub[1:-1, :-2] & sub[1:-1, 2:])
        jy, ix = np.nonzero(edge)
        pts = np.stack([wx0 + (ix + 0.5) * res, wy0 + (jy + 0.5) * res], axis=1)
        tight = min(m_lat, 0.12)

        # Final pose: the nominal one, nudged just enough to clear what is actually there. The
        # maneuver ends with a straight run along the stall axis (the docking run), so the
        # search aims at the start of that run.
        th = nominal[2]
        h = np.array([math.cos(th), math.sin(th)])
        if spec["kind"] == "manual":
            u, nu = -h, np.array([-h[1], h[0]])            # trials: a sideways, b along the box
        else:
            u, nu = spec["u"], np.array([-spec["u"][1], spec["u"][0]])   # b: back out of the stall
        best = None
        for sign in spec["signs"]:
            goal = None
            for _, a, b in spec["trials"]:
                p = (nominal[0] + a * nu[0] - b * u[0], nominal[1] + a * nu[1] - b * u[1], th)
                if footprint_hits(np.array([p]), pts, tight)[0] or sub[
                        min(max(int((p[1] - wy0) / res), 0), sub.shape[0] - 1),
                        min(max(int((p[0] - wx0) / res), 0), sub.shape[1] - 1)]:
                    continue
                for dock in spec["runs"]:
                    q = (p[0] - sign * dock * h[0], p[1] - sign * dock * h[1], th)
                    n = int(dock / 0.2) + 1
                    line = np.stack([np.linspace(q[0], p[0], n), np.linspace(q[1], p[1], n), np.full(n, th)], axis=1)
                    if cs.query(*q) != CSpace.HARD and not footprint_hits(line, pts, tight).any():
                        goal, pre = p, q
                        break
                if goal is not None:
                    break
            if goal is None:
                continue
            h2d, cell = holonomic_distance(sub, res, pre[:2], wx0, wy0)
            docking = dict(goal=goal, sign=sign, length=dock) if dock > 0.0 else None
            rows = self.planner.search(start, pre, cs, h2d, cell, docking)
            if rows is None:
                continue
            stats = dict(self.planner.stats)
            segs = split_segments(start, rows) if len(rows) else []
            if any(footprint_hits(sg.poses(), pts, 0.03).any() for sg in segs):
                continue
            if best is None or stats["cost"] < best["stats"]["cost"]:
                best = dict(segments=segs, goal=goal, nominal=nominal, margin=m_lat, stats=stats,
                            explored=self.planner.explored)
        return best

    def _plan_done(self):
        self.plan_time += time.time() - self._plan_wall
        res = self.plan_result
        if res is None:
            if self.must_replan:
                self._finish(False, "the way is blocked and there is no other maneuver")
                return
            if self.path and self.seg_i < len(self.path):
                self.say("no better plan found, continuing with the current one")
                self.state = "DRIVE"
                self.gear_changes += 1
                self._follow(self.path[self.seg_i])
                return
            if self.path:      # a final correction was not possible: stay where we are
                self._finish(True, "parked (no room to correct further)")
                return
            if self.manual is not None:
                self._finish(False, "cannot reach that spot (blocked, or not seen to be free yet)")
                return
            here = np.array(self.pose[:2]) + EGO.center * self.travel_dir
            old = [r for r in self.rejected if np.hypot(*(self.target.center - r[0])) < 1.5]
            self.rejected = [r for r in self.rejected if r not in old]
            self.rejected.append((self.target.center.copy(), here, 1 + sum(r[2] for r in old)))
            self.target = None
            self.say("no feasible maneuver into that stall, searching on")
            self.state = "SEARCH"
            self._follow(self._search_route(), presteer=False)
            return
        segs = res["segments"]
        self.goal, self.nominal = res["goal"], res["nominal"]
        self.blocked, self.must_replan = 0, False
        self.plan_info = dict(res["stats"], margin=res["margin"], explored=res["explored"])
        st = res["stats"]
        self.say("plan: %s  (%d expansions, %.1f s, margin %.2f m)" % (
            " + ".join("%s %.1f m" % ("fwd" if s.dir > 0 else "rev", s.length) for s in segs) or "already there",
            st["iterations"], time.time() - self._plan_wall, res["margin"]))
        self.path, self.seg_i = segs, 0
        if not segs:
            self._finish(True, "parked")
            return
        self.state = "DRIVE"
        self._follow(segs[0])

    # ---- execution -------------------------------------------------------------

    def _retarget(self):
        """Follow the chosen stall through the stream of fresh stall estimates."""
        cand = [s for s in self.slots if s.kind == self.target.kind and
                np.hypot(*(s.center - self.target.center)) < 1.2]
        if cand:
            self.target = min(cand, key=lambda s: np.hypot(*(s.center - self.target.center)))
        return bool(cand)

    def _refine(self):
        """Keep the plan attached to the stall as its line estimates improve."""
        if not self._retarget():
            return
        new = self.target.goal(self.nose_in, self.park_dir)
        old = self.nominal
        d, dth = math.hypot(new[0] - old[0], new[1] - old[1]), abs(wrap(new[2] - old[2]))
        if d < 0.01 and dth < 0.003:
            return
        if d > 0.5 or dth > 0.1:
            if self.replans < self.MAX_REPLANS:
                self.replans += 1
                self.say("the stall estimate jumped, replanning")
                self.tracker.stop()
                self.state, self.t_still = "BRAKE", None
            return
        a = 0.3                       # move the plan gradually so the tracker is not jerked around
        new = (old[0] + a * (new[0] - old[0]), old[1] + a * (new[1] - old[1]),
               old[2] + a * wrap(new[2] - old[2]))
        beyond = 0.0
        for seg in reversed(self.path[self.seg_i:]):
            seg.reanchor(old, new, beyond)
            beyond += seg.length
        c, s = math.cos(new[2] - old[2]), math.sin(new[2] - old[2])
        dx, dy = self.goal[0] - old[0], self.goal[1] - old[1]
        self.goal = (new[0] + dx * c - dy * s, new[1] + dx * s + dy * c, wrap(self.goal[2] + new[2] - old[2]))
        self.nominal = new

    def _monitor(self):
        """Stop and replan if newly seen obstacles are in the way of the remaining path."""
        pts = self.grid.occupied_points()
        if len(pts) == 0:
            return
        poses = [self.path[self.seg_i].poses(self.tracker.i)] + [s.poses() for s in self.path[self.seg_i + 1:]]
        poses = np.concatenate(poses)
        near = pts[np.hypot(pts[:, 0] - self.pose[0], pts[:, 1] - self.pose[1]) < 25.0]
        self.blocked = self.blocked + 1 if footprint_hits(poses, near, 0.0).any() else 0
        if self.blocked >= 2:
            self.replans += 1
            self.must_replan = True            # the current plan cannot be driven any further
            self.say("the path is blocked by something newly seen, replanning")
            self.tracker.stop()
            self.state, self.t_still = "BRAKE", None

    def _segment_done(self):
        self.seg_i += 1
        if self.seg_i < len(self.path):
            self.gear_changes += 1
            self._follow(self.path[self.seg_i])
            return
        lon, lat, dth = self._pose_error(self.goal)
        if (abs(lat) > 0.12 or abs(dth) > math.radians(2.5) or abs(lon) > 0.3) and \
                self.corrections < self.MAX_CORRECTIONS:
            self.corrections += 1
            self.say("off the stall centre by %.2f m / %.1f deg, correcting" % (lat, math.degrees(dth)))
            self._request_plan()
            return
        self._finish(True, "parked")

    def _pose_error(self, ref):
        dx, dy = self.pose[0] - ref[0], self.pose[1] - ref[1]
        c, s = math.cos(ref[2]), math.sin(ref[2])
        return dx * c + dy * s, -dx * s + dy * c, wrap(self.pose[2] - ref[2])

    def _finish(self, ok, text):
        self.state = "PARKED" if ok else "FAILED"
        self.say(text)
        res = dict(ok=ok, time=self.time, plan_time=self.plan_time, gear_changes=self.gear_changes,
                   replans=self.replans, corrections=self.corrections, min_clearance=self.min_clearance)
        if ok and self.manual is not None and self.target is None:
            lon, lat, dth = self._pose_error((self.manual[0] - EGO.center * math.cos(self.manual[2]),
                                              self.manual[1] - EGO.center * math.sin(self.manual[2]), self.manual[2]))
            res.update(kind="manual", lateral=lat, depth=lon, heading_deg=math.degrees(abs(dth)))
            res["ok"] = bool(self.min_clearance > 0.0 and math.hypot(lon, lat) < 0.5)
        elif ok:
            # score against the ground-truth stall the car ended up in
            c = np.array(self.pose[:2]) + EGO.center * np.array([math.cos(self.pose[2]), math.sin(self.pose[2])])
            stall = min(self.scn.stalls, key=lambda s: np.hypot(*(s["center"] - c)))
            u = stall["u_in"]
            nu = np.array([-u[1], u[0]])
            axis = math.atan2(u[1], u[0]) + (0.5 * math.pi if stall["kind"] == "parallel" else 0.0)
            head = wrap(self.pose[2] - axis)
            head = min(abs(head), abs(wrap(head - math.pi)))
            poly = stall["corners"]
            e = np.roll(poly, -1, axis=0) - poly
            cr = lambda p: e[:, 0] * (p[1] - poly[:, 1]) - e[:, 1] * (p[0] - poly[:, 0])
            inside = all(np.all(cr(p) >= -0.02) or np.all(cr(p) <= 0.02) for p in ego_poly(self.pose))
            res.update(kind=stall["kind"], gt_occupied=stall["occupied"], inside_lines=bool(inside),
                       lateral=float((c - stall["center"]) @ (u if stall["kind"] == "parallel" else nu)),
                       depth=float((c - stall["center"]) @ (nu if stall["kind"] == "parallel" else u)),
                       heading_deg=math.degrees(head))
            res["ok"] = bool(inside and not stall["occupied"] and self.min_clearance > 0.0)
        if "lateral" in res:
            self.message = "%s: %.0f cm off centre, %.1f deg off axis, %d gear changes, %.0f s" % (
                text, 100.0 * abs(res["lateral"]), res["heading_deg"], self.gear_changes, self.time)
        self.result = res
        print("[result] " + "  ".join("%s=%s" % (k, ("%.3f" % v) if isinstance(v, float) else v)
                                        for k, v in res.items()), flush=True)


# =============================================================================
# Visualization: one Irrlicht window split into four live camera views, with
# the perception, the stall map and the plan drawn into the scene
# =============================================================================

_GLYPHS = {
    "A": "0E11111F111111", "B": "1E11111E11111E", "C": "0E11101010110E", "D": "1E11111111111E",
    "E": "1F10101E10101F", "F": "1F10101E101010", "G": "0E11101711110F", "H": "1111111F111111",
    "I": "0E04040404040E", "J": "0702020202120C", "K": "11121418141211", "L": "1010101010101F",
    "M": "111B1515111111", "N": "11191513111111", "O": "0E11111111110E", "P": "1E11111E101010",
    "Q": "0E11111115120D", "R": "1E11111E141211", "S": "0F10100E01011E", "T": "1F040404040404",
    "U": "1111111111110E", "V": "11111111110A04", "W": "1111111515150A", "X": "11110A040A1111",
    "Y": "11110A04040404", "Z": "1F01020408101F", "0": "0E11131519110E", "1": "040C040404040E",
    "2": "0E11010204081F", "3": "1E01010E01011E", "4": "02060A121F0202", "5": "1F101E0101110E",
    "6": "0608101E11110E", "7": "1F010204080808", "8": "0E11110E11110E", "9": "0E11110F01020C",
    ".": "00000000000C0C", ":": "000C0C000C0C00", "-": "0000001F000000", "+": "0004041F040400",
    "/": "01010204081010", "(": "02040808080402", ")": "08040202020408", ",": "000000000C0408",
    "<": "02040810080402", ">": "08040201020408", "=": "00001F001F0000", "%": "18190204081303",
    "'": "04040800000000", "!": "04040404040004", "?": "0E110102040004",
}
_RUNS = {}
for _ch, _hx in _GLYPHS.items():
    _runs = []
    for _row in range(7):
        _bits = int(_hx[2 * _row:2 * _row + 2], 16)
        _col = 0
        while _col < 5:
            if _bits & (16 >> _col):
                _c0 = _col
                while _col < 5 and _bits & (16 >> _col):
                    _col += 1
                _runs.append((_row, _c0, _col))
            else:
                _col += 1
    _RUNS[_ch] = _runs


class MouseKeys:
    """Polls the mouse and keyboard on macOS (Quartz + the Objective-C runtime through ctypes).
    PyChrono's Irrlicht bindings cannot deliver window events to Python, so this is how the
    drag-the-target mode gets its input."""
    KEYS = dict(space=49, enter=36, q=12, e=14, r=15, left=123, right=124)

    def __init__(self):
        import ctypes
        import ctypes.util
        if sys.platform != "darwin":
            raise RuntimeError("mouse/keyboard polling is only implemented for macOS")
        self.ct = ctypes
        cg = ctypes.CDLL("/System/Library/Frameworks/ApplicationServices.framework/ApplicationServices")
        cg.CGEventSourceButtonState.restype = ctypes.c_bool
        cg.CGEventSourceButtonState.argtypes = [ctypes.c_int32, ctypes.c_uint32]
        cg.CGEventSourceKeyState.restype = ctypes.c_bool
        cg.CGEventSourceKeyState.argtypes = [ctypes.c_int32, ctypes.c_uint16]
        self.cg = cg
        objc = ctypes.CDLL(ctypes.util.find_library("objc"))
        objc.objc_getClass.restype = ctypes.c_void_p
        objc.objc_getClass.argtypes = [ctypes.c_char_p]
        objc.sel_registerName.restype = ctypes.c_void_p
        objc.sel_registerName.argtypes = [ctypes.c_char_p]
        self.objc = objc

        class Point(ctypes.Structure):
            _fields_ = [("x", ctypes.c_double), ("y", ctypes.c_double)]
        self.Point = Point
        self.app = self._msg(objc.objc_getClass(b"NSApplication"), b"sharedApplication")
        self.window = None

    def _msg(self, obj, sel, restype=None, *args, argtypes=()):
        ct = self.ct
        fn = ct.cast(self.objc.objc_msgSend, ct.CFUNCTYPE(restype or ct.c_void_p, ct.c_void_p, ct.c_void_p, *argtypes))
        return fn(obj, self.objc.sel_registerName(sel), *args)

    def poll(self, height):
        """Mouse position in window pixels (origin top left), buttons, held keys, focus."""
        ct = self.ct
        if self.window is None:
            wins = self._msg(self.app, b"windows")
            if wins and self._msg(wins, b"count", ct.c_ulong):
                self.window = self._msg(wins, b"objectAtIndex:", None, 0, argtypes=(ct.c_ulong,))
        if not self.window:
            return None
        p = self._msg(self.window, b"mouseLocationOutsideOfEventStream", self.Point)
        return dict(x=p.x, y=height - p.y,
                    left=bool(self.cg.CGEventSourceButtonState(0, 0)),
                    right=bool(self.cg.CGEventSourceButtonState(0, 1)),
                    keys={k for k, code in self.KEYS.items() if self.cg.CGEventSourceKeyState(0, code)},
                    focus=bool(self._msg(self.app, b"isActive", ct.c_bool)))


class Viewer:
    COLORS = dict(det=(1.0, 0.9, 0.1), track=(0.1, 0.9, 1.0), free=(0.2, 1.0, 0.3), occupied=(1.0, 0.25, 0.2),
                  unknown=(0.6, 0.6, 0.6), fwd=(0.3, 0.55, 1.0), rev=(1.0, 0.35, 0.9), goal=(1.0, 1.0, 1.0),
                  scan=(1.0, 0.45, 0.1), box=(1.0, 1.0, 1.0), box_bad=(1.0, 0.2, 0.2), mpc=(1.0, 0.95, 0.2))

    def __init__(self, sim, args):
        global irr
        import pychrono.irrlicht as irr
        self.sim, self.args = sim, args
        self.W, self.H = args.window
        self.drag = args.target == "drag"
        vis = irr.ChVisualSystemIrrlicht()
        vis.AttachSystem(sim.world.system)
        vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
        vis.SetWindowSize(self.W, self.H)
        vis.SetWindowTitle("Chrono automated parking")
        vis.Initialize()
        vis.AddSkyBox()
        vis.AddLightDirectional(60, 60, chrono.ChColor(0.55, 0.55, 0.55), chrono.ChColor(0.2, 0.2, 0.2),
                                chrono.ChColor(0.9, 0.9, 0.9))
        vis.AddLightDirectional(50, 240, chrono.ChColor(0.0, 0.0, 0.0), chrono.ChColor(0.0, 0.0, 0.0),
                                chrono.ChColor(0.45, 0.45, 0.45))
        self.vis = vis
        self.smgr, self.drv, self.dev = vis.GetSceneManager(), vis.GetVideoDriver(), vis.GetDevice()
        self.cams = []
        for _ in range(4):
            vis.AddCamera(chrono.ChVector3d(0, -10, 10), chrono.ChVector3d(0, 0, 0))
            cam = vis.GetActiveCamera()
            cam.setInputReceiverEnabled(False)
            cam.setNearValue(0.15)
            cam.setFarValue(400.0)
            self.cams.append(cam)
        vis.BeginScene()      # one stock frame: this is what hides Chrono's own info panel
        vis.Render()
        vis.EndScene()

        self.PW = 0 if args.no_panel else 380          # width of the internals panel
        W, H, top = self.W - self.PW, self.H, 30
        if args.layout == "wide":      # big top view on the left, three views stacked on the right
            xs, hh = int(0.64 * W), (H - top) // 3
            self.rects = [(0, top, xs, H), (xs, top, W, top + hh), (xs, top + hh, W, top + 2 * hh),
                          (xs, top + 2 * hh, W, H)]
        else:
            xm, ym = W // 2, top + (H - top) // 2
            self.rects = [(0, top, xm, ym), (xm, top, W, ym), (0, ym, xm, H), (xm, ym, W, H)]
        self.labels = ["TOP VIEW", "CHASE CAMERA", "FRONT CAMERA", "STALL CAMERA"]

        x0, y0, x1, y1 = sim.scn.bounds
        r = self.rects[0]
        aspect = (r[2] - r[0]) / (r[3] - r[1])
        self.top_alt = 150.0
        if self.drag:                  # whole lot, fixed, so the mouse maps to the ground
            self.top_c = np.array([0.5 * (x0 + x1), 0.5 * (y0 + y1)])
            self.top_half = max(0.5 * (y1 - y0) + 2.0, (0.5 * (x1 - x0) + 1.0) / aspect)
        else:
            self.top_c = np.array(sim.pose[:2])
            self.top_half = 17.0 / aspect
        self.chase_th = sim.pose[2]
        self.items, self.items_step = [], -1
        self.rect_cache = {}
        self.hist = collections.deque(maxlen=160)      # 16 s of signals for the strip charts
        self.hist_step = -10 ** 9
        self.map_img, self.map_time = None, -1.0       # cached raster of the planning map
        self.panel_rects = None
        self.next_shot = 0.0
        self.t_done = None

        self.box = None
        if self.drag:
            x, y, th = sim.pose
            self.box = [x + 12.0, y - 4.5, th - 0.5 * math.pi]
            self.box_ok = True
            self.grab = None
            self.box_t = 0.0               # when the box was last moved
            self.prev = dict(left=False, right=False, keys=set(), x=0.0)
            self.buttons = [("GO", (r[0] + 12, r[1] + 30, r[0] + 82, r[1] + 58)),
                            ("<", (r[0] + 92, r[1] + 30, r[0] + 126, r[1] + 58)),
                            (">", (r[0] + 132, r[1] + 30, r[0] + 166, r[1] + 58)),
                            ("FLIP", (r[0] + 176, r[1] + 30, r[0] + 250, r[1] + 58))]
            try:
                self.input = MouseKeys()
            except Exception as exc:
                self.input = None
                print("[parking] no interactive input (%s); use --target X,Y,DEG instead" % exc)

    # ---- drawing helpers -------------------------------------------------------

    def _vec(self, pts, z):
        v = chrono.vector_ChVector3d()
        for px, py in pts:
            v.push_back(chrono.ChVector3d(float(px), float(py), z))
        return v

    def _line(self, pts, color, z=0.05, thick=3, closed=False):
        """Queue a ground polyline; 'thick' parallel copies make it readable from far away."""
        pts = np.asarray(pts, dtype=float)
        if closed:
            pts = np.vstack([pts, pts[:1]])
        if len(pts) < 2:
            return
        col = chrono.ChColor(*color)
        if thick == 1:
            self.items.append((self._vec(pts, z), col))
            return
        d = np.gradient(pts, axis=0)
        n = np.stack([-d[:, 1], d[:, 0]], axis=1)
        n /= np.maximum(np.hypot(n[:, 0], n[:, 1])[:, None], 1e-9)
        for k in range(thick):
            self.items.append((self._vec(pts + n * 0.04 * (k - 0.5 * (thick - 1)), z), col))

    def _rect(self, color, x0, y0, x1, y1):
        key = (x0, y0, x1, y1)
        r = self.rect_cache.get(key)
        if r is None:
            if len(self.rect_cache) > 20000:
                self.rect_cache.clear()
            r = self.rect_cache[key] = irr.recti(int(x0), int(y0), int(x1), int(y1))
        self.drv.draw2DRectangle(color, r)

    def _text(self, text, x, y, scale=2, rgb=(255, 255, 255), alpha=255):
        col = irr.SColor(alpha, *rgb)
        for ch in text.upper():
            for row, c0, c1 in _RUNS.get(ch, ()):
                self._rect(col, x + c0 * scale, y + row * scale, x + c1 * scale, y + (row + 1) * scale)
            x += 6 * scale

    # ---- scene overlays --------------------------------------------------------

    def _rebuild_items(self):
        sim, C = self.sim, self.COLORS
        self.items = []
        for x1, y1, x2, y2, _ in sim.dets:
            self._line([(x1, y1), (x2, y2)], C["det"], z=0.07, thick=1)
        for t in sim.lines.confirmed():
            self._line(t.ends(), C["track"], z=0.04)
        for s in sim.slots:
            if s is sim.target:
                continue
            self._line(s.corners, C[s.status], z=0.03, thick=1, closed=True)
            if s.status == Slot.OCCUPIED:
                self._line([s.corners[0], s.corners[2]], C[s.status], z=0.03, thick=1)
                self._line([s.corners[1], s.corners[3]], C[s.status], z=0.03, thick=1)
        if sim.target is not None:
            self._line(sim.target.corners, C["free"], z=0.06, thick=5, closed=True)
        for i, seg in enumerate(sim.path[sim.seg_i:] if sim.state in ("DRIVE", "BRAKE", "PLAN") else []):
            i0 = sim.tracker.i if i == 0 and sim.state == "DRIVE" else 0
            self._line(np.stack([seg.x[i0::3], seg.y[i0::3]], axis=1), C["fwd"] if seg.dir > 0 else C["rev"], z=0.08)
        if sim.goal is not None and sim.path:
            self._line(ego_poly(sim.goal), C["goal"], z=0.08, closed=True)
        if sim.state in ("DRIVE", "SEARCH") and len(sim.tracker.horizon):
            self._line(sim.tracker.horizon, C["mpc"], z=0.11)
        if len(sim.scan):
            p = sim.scan
            cut = np.flatnonzero(np.hypot(*np.diff(p, axis=0).T) > 0.6) + 1
            for run in np.split(p, cut):
                if len(run) >= 2:
                    self.scan_items.append((self._vec(run, 0.5), chrono.ChColor(*C["scan"])))

    def _box_items(self):
        x, y, th = self.box
        col = self.COLORS["box"] if self.box_ok else self.COLORS["box_bad"]
        half = 0.5 * EGO.length
        poly = rect_poly(x, y, th, -half, half, EGO.half_width)
        c, s = math.cos(th), math.sin(th)
        nose = [(x + (half - 1.2) * c - 0.6 * s, y + (half - 1.2) * s + 0.6 * c), (x + half * c, y + half * s),
                (x + (half - 1.2) * c + 0.6 * s, y + (half - 1.2) * s - 0.6 * c)]
        keep = self.items
        self.items = []
        self._line(poly, col, z=0.12, thick=5, closed=True)
        self._line(nose, col, z=0.12, thick=3)
        out, self.items = self.items, keep
        return out

    # ---- cameras ---------------------------------------------------------------

    def _cameras(self):
        sim = self.sim
        x, y, th = sim.pose
        c, s = math.cos(th), math.sin(th)
        cx, cy = x + EGO.center * c, y + EGO.center * s
        V = irr.vector3df

        if not self.drag:
            self.top_c += 0.08 * (np.array([cx, cy]) - self.top_c)
        cam = self.cams[0]
        cam.setPosition(V(float(self.top_c[0]), float(self.top_c[1]) - 0.01, self.top_alt))
        cam.setTarget(V(float(self.top_c[0]), float(self.top_c[1]), 0.0))
        cam.setFOV(2.0 * math.atan(self.top_half / self.top_alt))

        self.chase_th += 0.06 * wrap(th - self.chase_th)
        cc, cs_ = math.cos(self.chase_th), math.sin(self.chase_th)
        cam = self.cams[1]
        cam.setPosition(V(cx - 9.5 * cc, cy - 9.5 * cs_, 4.6))
        cam.setTarget(V(cx + 3.0 * cc, cy + 3.0 * cs_, 0.6))
        cam.setFOV(0.95)

        cam = self.cams[2]
        reverse = sim.state == "DRIVE" and sim.tracker.seg is not None and sim.tracker.seg.dir < 0
        self.labels[2] = "REAR CAMERA" if reverse else "FRONT CAMERA"
        if reverse:
            ex, ey = x - (EGO.rear + 0.05) * c, y - (EGO.rear + 0.05) * s
            cam.setPosition(V(ex, ey, 1.0))
            cam.setTarget(V(ex - 4.0 * c, ey - 4.0 * s, -0.4))
        else:
            ex, ey = x + (EGO.front + 0.05) * c, y + (EGO.front + 0.05) * s
            cam.setPosition(V(ex, ey, 0.85))
            cam.setTarget(V(ex + 5.0 * c, ey + 5.0 * s, 0.0))
        cam.setFOV(1.25)

        cam = self.cams[3]
        goal = sim.target.center if sim.target is not None else (np.array(sim.manual[:2]) if sim.manual else None)
        if goal is not None and sim.state in ("BRAKE", "PLAN", "DRIVE", "PARKED"):
            u = sim.target.u_in if sim.target is not None else -np.array([math.cos(sim.manual[2]), math.sin(sim.manual[2])])
            eye = goal + 9.0 * u + 3.5 * np.array([-u[1], u[0]])
            aim = goal - 2.0 * u
            self.labels[3] = "STALL CAMERA"
            cam.setPosition(V(float(eye[0]), float(eye[1]), 5.5))
            cam.setTarget(V(float(aim[0]), float(aim[1]), 0.3))
        else:
            self.labels[3] = "SIDE CAMERA"
            cam.setPosition(V(cx - 4.0 * c - 11.0 * s, cy - 4.0 * s + 11.0 * c, 5.5))
            cam.setTarget(V(cx + 2.0 * c, cy + 2.0 * s, 0.4))
        cam.setFOV(0.85)

    # ---- drag-the-target mode --------------------------------------------------

    def _to_world(self, mx, my):
        r = self.rects[0]
        w, h = r[2] - r[0], r[3] - r[1]
        return np.array([self.top_c[0] + ((mx - r[0]) / w - 0.5) * 2.0 * self.top_half * w / h,
                         self.top_c[1] - ((my - r[1]) / h - 0.5) * 2.0 * self.top_half])

    def _to_px(self, p):
        r = self.rects[0]
        w, h = r[2] - r[0], r[3] - r[1]
        return (r[0] + ((p[0] - self.top_c[0]) / (2.0 * self.top_half * w / h) + 0.5) * w,
                r[1] + (0.5 - (p[1] - self.top_c[1]) / (2.0 * self.top_half)) * h)

    def handle_input(self, m, dt):
        """Move the target box with the mouse state m (see MouseKeys.poll); returns the action taken."""
        sim, prev, action = self.sim, self.prev, None
        if m is None or not m["focus"]:       # only listen while this window is in front
            self.grab = None
            return None
        r = self.rects[0]
        inside = r[0] <= m["x"] < r[2] and r[1] <= m["y"] < r[3]
        click = m["left"] and not prev["left"]
        hit = None
        if click:
            hit = next((name for name, (bx0, by0, bx1, by1) in self.buttons
                        if bx0 <= m["x"] < bx1 and by0 <= m["y"] < by1), None)
        pressed = m["keys"] - prev["keys"]
        if hit == "GO" or pressed & {"space", "enter"}:
            action = "go"
        elif hit == "FLIP" or "r" in pressed:
            self.box[2] = wrap(self.box[2] + math.pi)
        elif hit in ("<", ">"):
            self.box[2] = wrap(self.box[2] + math.radians(15.0 if hit == "<" else -15.0))
        elif click and inside:
            p = self._to_world(m["x"], m["y"])
            off = np.array(self.box[:2]) - p
            self.grab = off if np.hypot(*off) < 3.5 else np.zeros(2)     # grab the box, or jump it here
        if m["keys"] & {"q", "left"}:
            self.box[2] = wrap(self.box[2] + 1.2 * dt)
        if m["keys"] & {"e", "right"}:
            self.box[2] = wrap(self.box[2] - 1.2 * dt)
        if m["right"] and inside:
            if prev["right"]:
                self.box[2] = wrap(self.box[2] - 0.012 * (m["x"] - prev["x"]))
        if self.grab is not None:
            if m["left"]:
                p = self._to_world(m["x"], m["y"]) + self.grab
                x0, y0, x1, y1 = sim.scn.bounds
                self.box[0], self.box[1] = min(max(p[0], x0), x1), min(max(p[1], y0), y1)
            else:
                self.grab = None
                if not self.args.no_snap:      # dropped on a stall the map knows: line up with it
                    h = np.array([math.cos(self.box[2]), math.sin(self.box[2])])
                    for s in sim.slots:
                        axis = s.along if s.kind == "parallel" else s.u_in
                        if np.hypot(*(s.center - np.array(self.box[:2]))) < 1.2 and abs(axis @ h) > 0.9:
                            a = axis if axis @ h > 0 else -axis
                            self.box = [float(s.center[0]), float(s.center[1]), math.atan2(a[1], a[0])]
        self.prev = dict(left=m["left"], right=m["right"], keys=set(m["keys"]), x=m["x"])
        if self.grab is not None or hit or m["right"] or m["keys"]:
            self.box_t = time.time()
        pts = sim.grid.occupied_points()
        pose = np.array([[self.box[0] - EGO.center * math.cos(self.box[2]),
                          self.box[1] - EGO.center * math.sin(self.box[2]), self.box[2]]])
        self.box_ok = not footprint_hits(pose, pts, 0.05)[0]
        if action == "go" and sim.state != "PLAN":
            sim.go_to(tuple(self.box))
        return action

    # ---- frame -----------------------------------------------------------------

    def render(self):
        sim, drv = self.sim, self.drv
        if sim.steps != self.items_step:
            self.items_step = sim.steps
            self.scan_items = []
            self._rebuild_items()
        # the target box is shown while it is being placed, not while the car is carrying out a plan
        idle = sim.state in ("SETTLE", "WAIT", "PARKED", "FAILED")
        box_items = self._box_items() if self.box is not None and (idle or time.time() - self.box_t < 2.5) else []
        self._cameras()
        self.vis.BeginScene()
        for i, (cam, r) in enumerate(zip(self.cams, self.rects)):
            drv.setViewPort(irr.recti(*r))
            cam.setAspectRatio((r[2] - r[0]) / (r[3] - r[1]))
            self.smgr.setActiveCamera(cam)
            self.smgr.drawAll()
            for vec, col in self.items + box_items + (self.scan_items if i == 0 else []):
                irr.DrawPolyline(self.vis, vec, col, True)
        drv.setViewPort(irr.recti(0, 0, self.W, self.H))
        self._hud()
        self.vis.EndScene()

    def _hud(self):
        sim, W, H = self.sim, self.W, self.H
        dark, edge = irr.SColor(255, 18, 20, 24), irr.SColor(255, 8, 8, 10)
        self._rect(dark, 0, 0, W, 30)
        for x0, y0, x1, y1 in self.rects:              # frames around the views
            self._rect(edge, x0, y0, x1, y0 + 2); self._rect(edge, x0, y1 - 2, x1, y1)
            self._rect(edge, x0, y0, x0 + 2, y1); self._rect(edge, x1 - 2, y0, x1, y1)
        for label, r in zip(self.labels, self.rects):
            self._rect(irr.SColor(150, 0, 0, 0), r[0] + 8, r[1] + 8, r[0] + 20 + 12 * len(label), r[1] + 28)
            self._text(label, r[0] + 14, r[1] + 11)
        state = {"SETTLE": "STARTING", "SEARCH": "SEARCHING", "BRAKE": "STOPPING", "PLAN": "PLANNING",
                 "PARKED": "PARKED", "FAILED": "FAILED", "WAIT": "WAITING FOR TARGET"}.get(sim.state)
        if sim.state == "SEARCH" and sim.manual is not None:
            state = "DRIVING TO TARGET"
        if sim.state == "DRIVE":
            seg = sim.tracker.seg
            state = "%s %d/%d" % ("FORWARD" if seg.dir > 0 else "REVERSE", sim.seg_i + 1, len(sim.path))
        rgb = {"PARKED": (90, 255, 110), "FAILED": (255, 90, 80), "PLANNING": (255, 220, 60)}.get(state, (255, 255, 255))
        self._text(state, 12, 8, rgb=rgb)
        self._text(sim.message[:(self.W - 580) // 12], 300, 8, rgb=(190, 200, 210))
        self._text("T %5.1f S   %+.1f M/S" % (sim.time, sim.speed), W - 260, 8)
        if sim.steps - self.hist_step >= int(round(PERCEPTION_DT / STEP)):
            self.hist_step = sim.steps
            trk = sim.tracker
            driving = sim.state in ("DRIVE", "SEARCH") and trk.seg is not None
            self.hist.append((trk.err[0] if driving else 0.0, trk.err[1] if driving else 0.0, sim.speed,
                              trk.v_cmd * trk.seg.dir if driving else 0.0, sim.cmd[0],
                              trk.gain.g[1], trk.gain.g[-1]))
            self.panel_rects = None

        steer, throttle, brake = sim.cmd
        x0, y0 = 14, H - 54
        self._rect(irr.SColor(150, 0, 0, 0), x0 - 6, y0 - 6, x0 + 250, y0 + 46)
        self._text("STEER", x0, y0, 1)
        self._rect(irr.SColor(255, 70, 70, 70), x0 + 50, y0, x0 + 240, y0 + 8)
        mid = x0 + 145
        self._rect(irr.SColor(255, 80, 170, 255), min(mid, mid - 95 * steer), y0, max(mid, mid - 95 * steer) + 1, y0 + 8)
        for k, (name, val, col) in enumerate((("GAS", throttle, (90, 230, 110)), ("BRAKE", brake, (240, 80, 70)))):
            y = y0 + 14 + 14 * k
            self._text(name, x0, y, 1)
            self._rect(irr.SColor(255, 70, 70, 70), x0 + 50, y, x0 + 240, y + 8)
            self._rect(irr.SColor(255, *col), x0 + 50, y, x0 + 50 + 190 * min(max(val, 0.0), 1.0), y + 8)

        r = self.rects[0]
        legend = (("LINE DETECTIONS", "det"), ("LINE MAP", "track"), ("FREE STALL", "free"), ("OCCUPIED", "occupied"),
                  ("PATH FWD", "fwd"), ("PATH REV", "rev"), ("RANGE SCAN", "scan"))
        x, y = r[0] + 270, r[3] - 18
        self._rect(irr.SColor(150, 0, 0, 0), x - 6, y - 5, min(x + 6 + sum(22 + 6 * len(n) for n, _ in legend), r[2] - 4), y + 12)
        for name, key in legend:
            if x + 22 + 6 * len(name) > r[2] - 6:
                break
            c = tuple(int(255 * v) for v in self.COLORS[key])
            self._rect(irr.SColor(255, *c), x, y, x + 10, y + 7)
            self._text(name, x + 14, y, 1)
            x += 22 + 6 * len(name)

        for sl in sim.slots:                           # label what the stall logic concluded
            name = "TARGET" if sl is sim.target else ("FREE" if sl.status == Slot.FREE else None)
            px, py = self._to_px(sl.center)
            if name and r[0] + 30 < px < r[2] - 30 and r[1] + 40 < py < r[3] - 30:
                self._rect(irr.SColor(170, 0, 0, 0), px - 3 * len(name) - 3, py - 6, px + 3 * len(name) + 3, py + 5)
                self._text(name, px - 3 * len(name), py - 4, 1, rgb=(120, 255, 140))
        if self.PW:
            self._panel()
        if self.drag:
            for name, (bx0, by0, bx1, by1) in self.buttons:
                go = name == "GO"
                self._rect(irr.SColor(255, 40, 150, 70) if go else irr.SColor(255, 60, 66, 78), bx0, by0, bx1, by1)
                self._text(name, (bx0 + bx1) // 2 - 6 * len(name), by0 + 7)
            self._text("DRAG THE BOX: LEFT MOUSE.  ROTATE: RIGHT-DRAG, Q/E OR < >.  GO: SPACE", r[0] + 262, r[1] + 40, 1)

    # ---- internals panel -------------------------------------------------------
    # Everything here is drawn with filled rectangles (the only 2D primitive the Python
    # bindings expose reliably): pixel rasters are run-length encoded into rectangles.

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
        g_now = trk.gain.g[trk.seg.dir] if trk.seg is not None else EGO.kappa
        if self.panel_rects is None:
            out = self._map_rects(x, map_y)
            hz = np.array(self.hist) if self.hist else np.zeros((0, 7))
            if sim.state in ("DRIVE", "SEARCH") and len(trk.k_plan):
                lim, n = 1.25 * max(g_now, EGO.kappa), len(trk.k_plan)
                ty, th = tops[0]
                for vals, rgb, thick in ((np.full(n, g_now), (150, 60, 60), 1), (np.full(n, -g_now), (150, 60, 60), 1),
                                         (trk.k_ref, (150, 158, 170), 1), (trk.k_plan, (255, 240, 60), 2)):
                    py = (ty + th - 2 - (np.clip(vals, -lim, lim) + lim) / (2 * lim) * (th - 3)).astype(int)
                    for k in range(n):
                        out.append((irr.SColor(255, *rgb), x + k * w // n, py[k], x + (k + 1) * w // n, py[k] + thick))
            if len(hz):
                self._trace(out, x, tops[1][0], w, tops[1][1], 100.0 * hz[:, 0], -12.0, 12.0, (90, 220, 255))
                self._trace(out, x, tops[1][0], w, tops[1][1], np.degrees(hz[:, 1]), -6.0, 6.0, (255, 170, 60))
                self._trace(out, x, tops[2][0], w, tops[2][1], hz[:, 3], -2.6, 2.6, (120, 126, 138))
                self._trace(out, x, tops[2][0], w, tops[2][1], hz[:, 2], -2.6, 2.6, (120, 255, 140))
                self._trace(out, x, tops[3][0], w, tops[3][1], np.full(len(hz), EGO.kappa), 0.0, 2.0 * EGO.kappa, (120, 126, 138))
                self._trace(out, x, tops[3][0], w, tops[3][1], hz[:, 5], 0.0, 2.0 * EGO.kappa, (90, 150, 255))
                self._trace(out, x, tops[3][0], w, tops[3][1], hz[:, 6], 0.0, 2.0 * EGO.kappa, (255, 110, 235))
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
        self._text("FWD %.3f" % trk.gain.g[1], x + w - 196, tops[3][0] - 12, 1, rgb=(90, 150, 255))
        self._text("REV %.3f" % trk.gain.g[-1], x + w - 132, tops[3][0] - 12, 1, rgb=(255, 110, 235))
        self._text("MODEL %.3f" % EGO.kappa, x + w - 68, tops[3][0] - 12, 1, rgb=grey)

        free = sum(sl.status == Slot.FREE for sl in sim.slots)
        occ = sum(sl.status == Slot.OCCUPIED for sl in sim.slots)
        rows = ["LINE TRACKS %d (%d CONFIRMED)" % (len(sim.lines.tracks), len(sim.lines.confirmed())),
                "STALLS %d: %d FREE, %d OCCUPIED" % (len(sim.slots), free, occ),
                "CAR %.2f X %.2f M, WHEELBASE %.2f M" % (EGO.length, 2.0 * EGO.half_width, EGO.wheelbase),
                "MIN CLEARANCE SO FAR %.2f M" % (sim.min_clearance if sim.min_clearance < 1e9 else 0.0)]
        if info is not None:
            rows.append("PLAN MARGIN %.2f M, %d SEGMENTS" % (info["margin"], len(sim.path)))
        for k, row in enumerate(rows):
            if y + 12 * k + 10 < self.H:
                self._text(row, x, y + 12 * k, 1, rgb=grey)

    def loop(self):
        sim, args = self.sim, self.args
        frame_steps = max(1, int(round(1.0 / (30.0 * STEP))))
        wall0, sim0 = time.time(), sim.time
        last = time.time()
        while self.vis.Run():
            running = sim.advance(frame_steps)
            now = time.time()
            if self.drag and self.input is not None:
                self.handle_input(self.input.poll(self.H), now - last)
            last = now
            self.render()
            if args.snapshots and sim.time >= self.next_shot and running:
                self.vis.WriteImageToFile(os.path.join(args.snapshots, "frame_%04d.png" % int(round(sim.time * 10))))
                self.next_shot = sim.time + args.snapshot_dt
            if not running:                 # planning: time is frozen; mostly yield to the planner thread
                time.sleep(0.1)
                wall0, sim0 = time.time(), sim.time
                continue
            ahead = (sim.time - sim0) / args.speed - (time.time() - wall0)
            if ahead > 0.0:
                time.sleep(ahead)
            elif ahead < -0.5:              # fell behind (slow machine): do not try to catch up
                wall0, sim0 = time.time(), sim.time
            if sim.result is not None and not self.drag:
                if self.t_done is None:
                    self.t_done = sim.time
                    if args.snapshots:
                        self.vis.WriteImageToFile(os.path.join(args.snapshots, "final.png"))
                if args.exit_after is not None and sim.time - self.t_done > args.exit_after:
                    break
            if sim.time > args.timeout and args.exit_after is not None:
                break


# =============================================================================
# Entry point
# =============================================================================

TOUR = [("perpendicular", "both"), ("perpendicular", "left"), ("perpendicular", "right"),
        ("perpendicular", "none"), ("angled", "both"), ("angled", "none"), ("parallel", "both"),
        ("parallel", "none")]


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Automated parking in Project Chrono: perpendicular, angled and parallel stalls.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--type", choices=("perpendicular", "angled", "parallel"), default="perpendicular",
                    help="kind of parking stalls")
    ap.add_argument("--cars", choices=("both", "left", "right", "none", "random"), default="both",
                    help="parked cars next to the free stall, as seen from the driving lane "
                         "looking into the stall (none = empty lot, lines only)")
    ap.add_argument("--side", choices=("right", "left"), default="right",
                    help="side of the lane the free stall is on")
    ap.add_argument("--angle", type=float, default=60.0, help="stall angle for --type angled [deg]")
    ap.add_argument("--park", choices=("auto", "forward", "reverse"), default="auto",
                    help="drive into perpendicular/angled stalls nose first or backwards")
    ap.add_argument("--target", default=None, metavar="drag | X,Y,DEG",
                    help="choose the spot yourself: 'drag' to place a box with the mouse in the top "
                         "view, or the pose of the middle of the car")
    ap.add_argument("--no-snap", action="store_true",
                    help="with --target: park exactly where the box is, do not align with a detected stall")
    ap.add_argument("--tire", choices=("tmeasy", "pac02"), default="tmeasy",
                    help="tire model of the simulated sedan (TMeasy or Pacejka 2002)")
    ap.add_argument("--noise", type=float, default=1.0, help="perception noise scale (0 = perfect)")
    ap.add_argument("--seed", type=int, default=1, help="random seed (layout details and noise)")
    ap.add_argument("--tour", action="store_true", help="play through a set of scenarios one after another")
    ap.add_argument("--headless", action="store_true", help="no window, run as fast as possible")
    ap.add_argument("--layout", choices=("quad", "wide"), default=None,
                    help="2x2 views, or a large top view with three small ones (default with --target drag)")
    ap.add_argument("--window", default="1600x930", help="window size")
    ap.add_argument("--no-panel", action="store_true", help="hide the internals panel next to the views")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed relative to real time")
    ap.add_argument("--exit-after", type=float, default=None,
                    help="close the window this many seconds after parking (default: stay open)")
    ap.add_argument("--snapshots", default=None, metavar="DIR", help="save a PNG of the window every --snapshot-dt")
    ap.add_argument("--snapshot-dt", type=float, default=2.0, help="simulated time between snapshots [s]")
    ap.add_argument("--timeout", type=float, default=240.0, help="give up after this much simulated time [s]")
    args = ap.parse_args(argv)
    args.window = tuple(int(v) for v in args.window.lower().split("x"))
    if args.target is not None and args.target != "drag":
        x, y, deg = (float(v) for v in args.target.split(","))
        args.target = (x, y, math.radians(deg))
    if args.layout is None:
        args.layout = "wide" if args.target == "drag" else "quad"
    if args.snapshots:
        os.makedirs(args.snapshots, exist_ok=True)
    return args


def main():
    args = parse_args()
    if args.tour:
        rest = [a for a in sys.argv[1:] if a != "--tour"]
        for kind, cars in TOUR:
            print("\n=== %s stalls, cars: %s ===" % (kind, cars), flush=True)
            code = subprocess.call([sys.executable, os.path.abspath(__file__)] + rest +
                                   ["--type", kind, "--cars", cars, "--exit-after", "4"])
            if code not in (0, 1):
                return code
        return 0
    sim = ParkingSim(args)
    if args.headless:
        if args.target == "drag":
            sys.exit("--target drag needs the window; use --target X,Y,DEG with --headless")
        while sim.result is None and sim.time < args.timeout:
            sim.advance(500)
        if sim.result is None:
            sim._finish(False, "timed out")
    else:
        viewer = Viewer(sim, args)
        code = 1
        try:
            viewer.loop()
            code = 0 if sim.result is not None and sim.result["ok"] else 1
        except BaseException:
            import traceback
            traceback.print_exc()
        # Destroying the Irrlicht visual system from Python segfaults here (after the window
        # has closed), so keep it alive and leave without running destructors
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)
    return 0 if sim.result is not None and sim.result["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
