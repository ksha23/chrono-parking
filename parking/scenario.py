"""Scenarios: the ground-truth layout of painted lines, stalls, parked cars and kerbs."""

import math
import os

import numpy as np

from .chrono_env import chrono
from .geometry import rect_poly


# Parked-car models, built from the vehicle meshes that ship with Chrono. Only the wheel hub
# positions (x, y, z of the front and rear left hubs, from Chrono's vehicle definitions) are
# listed here; the outline and ride height come from the meshes themselves. The outline is that
# of the body seen from above, taken below half height, which leaves out the mirrors. It is what
# the car collides with, what clearance is measured against, and what a sensor sees of it.
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


def _outline(pts, n=32):
    """Convex outline of a set of 2D points: the furthest point in each of n directions,
    counter-clockwise. Always n vertices, some of which may coincide."""
    a = np.arange(n) * (2.0 * math.pi / n)
    return pts[np.argmax(pts @ np.stack([np.cos(a), np.sin(a)]), axis=0)]


def parked_model(name):
    """A parked-car model with its extent (x0, x1, hw), its outline in the mesh frame, and its
    ride height z."""
    m = _PARKED_GEOMETRY.get(name)
    if m is None:
        m = dict(PARKED[name], x0=-2.45, x1=2.45, hw=1.05, z=0.25)
        m["outline"] = _outline(np.array([(-2.45, -1.05), (2.45, -1.05), (2.45, 1.05), (-2.45, 1.05)]))
        data = chrono.GetChronoDataPath() + "vehicle/"
        body, tire = data + m["mesh"] % m["colors"][0], data + m["tire"]
        if os.path.exists(body) and os.path.exists(tire):
            mesh = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(body, False, False)
            box = mesh.GetBoundingBox()
            wheel = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(tire, False, False).GetBoundingBox()
            hub = sum(w[2] for w in m["wheels"]) / len(m["wheels"])
            v = mesh.GetCoordsVertices()         # (indexed: iterating it ends in an uncaught exception on some builds)
            pts = np.array([(v[i].x, v[i].y, v[i].z) for i in range(len(v))])
            low = pts[pts[:, 2] < 0.5 * (box.min.z + box.max.z), :2]
            m.update(x0=box.min.x, x1=box.max.x, hw=float(np.abs(low[:, 1]).max()), outline=_outline(low),
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
        c, s = math.cos(yaw), math.sin(yaw)
        o = m["outline"] - (0.5 * (m["x0"] + m["x1"]), 0.0)          # about the middle of the car
        poly = np.stack([cx + o[:, 0] * c - o[:, 1] * s, cy + o[:, 0] * s + o[:, 1] * c], axis=1)
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
    scn = make_street(cars, sg, rng) if kind == "parallel" else \
        make_lot(90.0 if kind == "perpendicular" else angle, cars, sg, rng)
    scn.seed = seed
    return scn
