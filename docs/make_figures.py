#!/usr/bin/env python3
"""Regenerates the figures in docs/img from real runs of parking_sim.py.

Needs a Python that has PyChrono and matplotlib:

    python docs/make_figures.py

Three headless simulations are run one after another (about a minute in total), with the stand-in
perception. The figures of the sensors are made by make_sensor_figures.py."""

import math
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, "img")
os.makedirs(IMG, exist_ok=True)
sys.path.insert(0, os.path.join(HERE, ".."))
from parking.agent import ParkingSim
from parking.cli import parse_args
from parking.config import STEER_RATE
from parking.control import LateralMPC
from parking.geometry import ego_poly, rect_poly, wrap
from parking.mapping import LineMap
from parking.perception import Perception
from parking.planner import CSpace, holonomic_distance
from parking.reeds_shepp import rs_paths, rs_sample
from parking.scenario import make_scenario
from parking.vehicle import EGO

C = dict(fwd="#2f6fe0", rev="#d63ec8", free="#22a745", occ="#d9412b", track="#0aa5c4", det="#e0a800",
         car="#b02a2a", mpc="#d4a000", grey="#8a8f98", obstacle="#c9ccd1")
plt.rcParams.update({"font.size": 9, "axes.titlesize": 10, "figure.dpi": 110, "savefig.bbox": "tight"})


def save(fig, name):
    fig.savefig(os.path.join(IMG, name), dpi=130)
    plt.close(fig)
    print("wrote docs/img/" + name)


def outline(ax, poly, **kw):
    p = np.vstack([poly, poly[:1]])
    ax.plot(p[:, 0], p[:, 1], **kw)


def draw_scene(ax, scn, stalls=False):
    for x1, y1, x2, y2, col in scn.lines:
        ax.plot([x1, x2], [y1, y2], color="#f2c200" if col == "yellow" else "#9aa0a8", lw=1.6, solid_capstyle="butt", zorder=1)
    for poly in scn.obstacle_polys():
        ax.add_patch(Polygon(poly, closed=True, fc=C["obstacle"], ec="#6d7178", lw=0.6, zorder=2))
    if stalls:
        for s in scn.stalls:
            if not s["occupied"] and s["target_row"]:
                ax.add_patch(Polygon(s["corners"], closed=True, fc=C["free"], alpha=0.25, ec="none", zorder=1))
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


def args_for(kind, cars, seed=1, extra=()):
    old = sys.argv
    # these figures show the pipeline with the stand-in perception, which every PyChrono can run
    sys.argv = ["parking_sim.py", "--headless", "--sensors", "sim", "--type", kind, "--cars", cars, "--seed", str(seed), *extra]
    try:
        return parse_args()
    finally:
        sys.argv = old


def run(kind, cars, seed=1):
    """Run one scenario, recording what the figures need."""
    sim = ParkingSim(args_for(kind, cars, seed))
    rec = dict(sim=sim, t=[], pose=[], v=[], vcmd=[], steer=[], drive=[], brake=[], e=[], psi=[], gf=[], gr=[], state=[],
               frame=None, plan=None, mpc=None)
    while sim.result is None and sim.time < 200.0:
        sim.advance(25)
        trk = sim.tracker
        on = sim.state in ("DRIVE", "SEARCH") and trk.seg is not None
        rec["t"].append(sim.time)
        rec["pose"].append(sim.pose)
        rec["v"].append(sim.speed)
        rec["vcmd"].append(trk.v_cmd * trk.seg.dir if on else 0.0)
        rec["steer"].append(sim.cmd[0])
        rec["drive"].append(sim.cmd[1])
        rec["brake"].append(sim.cmd[2])
        rec["e"].append(trk.err[0] if on else np.nan)
        rec["psi"].append(trk.err[1] if on else np.nan)
        rec["gf"].append(trk.gain.g[1])
        rec["gr"].append(trk.gain.g[-1])
        rec["state"].append(sim.state)
        if rec["frame"] is None and sim.state == "SEARCH" and sim.pose[0] > 4.0 and len(sim.dets) >= 3:
            rec["frame"] = dict(pose=sim.pose, dets=list(sim.dets), scan=sim.scan.copy())
        if rec["plan"] is None and sim.plan_info is not None:
            occ = sim.grid.blocked()
            if sim.target is not None and sim.target.region is not None:
                mask, sl = sim.target.region
                occ[sl] &= ~mask | sim.grid.occupied()[sl]
            rec["plan"] = dict(pose=sim.pose, occ=occ, occupied=sim.grid.occupied().copy(), free=sim.grid.free.copy(),
                               segs=[(s.x.copy(), s.y.copy(), s.th.copy(), s.dir) for s in sim.path], goal=sim.goal,
                               explored=sim.plan_info["explored"].copy(), stats=dict(sim.plan_info),
                               tracks=[(t.ends(), t.hits) for t in sim.lines.tracks],
                               slots=[(s.corners.copy(), s.status, s.center.copy(), s.u_in.copy(), s is sim.target) for s in sim.slots])
        # an MPC solve in the middle of a curvature change makes the most instructive example
        if rec["mpc"] is None and sim.state == "DRIVE" and len(trk.k_ref) and np.ptp(trk.k_ref) > 0.1 and abs(sim.speed) > 0.5:
            i = trk.i
            rec["mpc"] = dict(k_ref=trk.k_ref.copy(), k_plan=trk.k_plan.copy(), g=trk.gain.g[trk.seg.dir], d=trk.seg.dir,
                              e=trk.err[0], psi=trk.err[1], steer=trk.delta, v=sim.speed, pose=sim.pose,
                              horizon=trk.horizon.copy(), seg=(trk.seg.x[i:i + 60].copy(), trk.seg.y[i:i + 60].copy()))
    for k in ("t", "v", "vcmd", "steer", "drive", "brake", "e", "psi", "gf", "gr"):
        rec[k] = np.array(rec[k])
    print("%s / %s: %s" % (kind, cars, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in sim.result.items()}))
    return rec


def extent(g):
    return (g.x0, g.x0 + g.nx * g.RES, g.y0, g.y0 + g.ny * g.RES)


# ------------------------------------------------------------------------------------------------
def fig_scenarios():
    fig, axes = plt.subplots(3, 1, figsize=(11, 11.5), gridspec_kw=dict(height_ratios=[1.0, 1.0, 0.55]))
    for ax, (kind, title) in zip(axes, (("perpendicular", "perpendicular stalls, 7.0 m two-way aisle"),
                                        ("angled", "60 degree stalls, 5.5 m one-way aisle"),
                                        ("parallel", "parallel stalls 7.2 m x 2.5 m along the kerb"))):
        scn = make_scenario(kind, "both", "right", 60.0, 1)
        draw_scene(ax, scn, stalls=True)
        x, y, th = scn.start
        ax.annotate("", xy=(scn.route_end, y), xytext=(x, y), arrowprops=dict(arrowstyle="->", color=C["fwd"], lw=1.2, ls="--"))
        ax.add_patch(Polygon(rect_poly(x, y, th, -2.45, 2.45, 0.925), closed=True, fc=C["car"], ec="k", lw=0.6, zorder=3))
        ax.set_title(title + "  (search route dashed, free stall green)")
        x0, y0, x1, y1 = scn.bounds
        ax.set_xlim(x0 + 2, x1 - 2)
        ax.set_ylim(y0 - 0.5, y1 + 0.5)
    save(fig, "scenarios.png")


def fig_perception(rec):
    sim, fr = rec["sim"], rec["frame"]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    for ax in axes:
        draw_scene(ax, sim.scn)
        outline(ax, ego_poly(fr["pose"]), color=C["car"], lw=1.5, zorder=5)
        ax.set_xlim(fr["pose"][0] - 13, fr["pose"][0] + 17)
        ax.set_ylim(-10.5, 10.5)
    ax = axes[0]
    for x1, y1, x2, y2, r in fr["dets"]:
        ax.plot([x1, x2], [y1, y2], color=C["det"], lw=2.0, zorder=4)
    ax.add_patch(plt.Circle((fr["pose"][0] + EGO.center * math.cos(fr["pose"][2]), fr["pose"][1]), Perception.LINE_RANGE,
                            fc="none", ec=C["grey"], ls=":", lw=0.8))
    ax.set_title("line detections of one frame (yellow): noisy, clipped by range and by occlusion, with clutter")
    ax = axes[1]
    ox, oy = fr["pose"][0] + EGO.center * math.cos(fr["pose"][2]), fr["pose"][1] + EGO.center * math.sin(fr["pose"][2])
    for px, py in fr["scan"][::3]:
        ax.plot([ox, px], [oy, py], color="#f08a24", lw=0.25, alpha=0.6, zorder=3)
    ax.plot(fr["scan"][:, 0], fr["scan"][:, 1], ".", color=C["occ"], ms=2.5, zorder=4)
    ax.set_title("range scan of the same frame: 360 rays, 16 m, 3 cm noise (every third ray drawn)")
    save(fig, "perception.png")


def fig_mapping(rec):
    sim, pl = rec["sim"], rec["plan"]
    g = sim.grid
    fig, axes = plt.subplots(2, 1, figsize=(12, 9))
    ax = axes[0]
    img = np.zeros(pl["occ"].shape + (3,))
    img[:] = (1.0, 1.0, 1.0)
    img[pl["free"] < 1] = (0.80, 0.82, 0.85)
    img[pl["occupied"]] = (0.85, 0.25, 0.17)
    ax.imshow(img, origin="lower", extent=extent(g), interpolation="nearest")
    for x1, y1, x2, y2, _ in sim.scn.lines:
        ax.plot([x1, x2], [y1, y2], color="#b9bdc4", lw=0.8)
    for (a, b), hits in pl["tracks"]:
        ax.plot([a[0], b[0]], [a[1], b[1]], color=C["track"] if hits >= LineMap.MIN_HITS else "#9fd8e6", lw=2.2 if hits >= 5 else 1.0)
    outline(ax, ego_poly(pl["pose"]), color=C["car"], lw=1.5)
    ax.set_title("map when the car stops to plan: seen free (white), never seen (grey), obstacle cells (red), line tracks (cyan)")
    ax = axes[1]
    ax.imshow(np.where(pl["occ"], 0.78, 1.0), origin="lower", extent=extent(g), cmap="gray", vmin=0, vmax=1, interpolation="nearest")
    for corners, status, center, u_in, is_target in pl["slots"]:
        col = dict(free=C["free"], occupied=C["occ"], unknown=C["grey"])[status]
        ax.add_patch(Polygon(corners, closed=True, fc=col, alpha=0.30 if not is_target else 0.55, ec=col, lw=1.2))
        ax.annotate("", xy=center + 1.6 * u_in, xytext=center - 0.2 * u_in, arrowprops=dict(arrowstyle="->", color=col, lw=1.2))
    outline(ax, ego_poly(pl["pose"]), color=C["car"], lw=1.5)
    outline(ax, ego_poly(pl["goal"]), color="k", lw=1.2, ls="--")
    ax.set_title("planning map (grey = blocked: obstacle or never seen) with the inferred stalls: free (green), occupied (red), "
                 "arrows point into the stall, dashed = goal pose")
    for ax in axes:
        ax.set_xlim(pl["pose"][0] - 16, pl["pose"][0] + 16)
        ax.set_ylim(g.y0, g.y0 + g.ny * g.RES)
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
    save(fig, "mapping.png")


def fig_cspace(rec):
    sim, pl = rec["sim"], rec["plan"]
    g = sim.grid
    cs = CSpace(pl["occ"], g.x0, g.y0, g.RES, 0.25, 0.30)
    fig, axes = plt.subplots(3, 1, figsize=(11, 10.5))
    for ax, deg in zip(axes, (0, 45, 90)):
        k = int(round(math.radians(deg) * cs.NTH / (2 * math.pi)))
        rgb = np.ones(cs.cost[k].shape + (3,))
        rgb[cs.cost[k] == cs.SOFT] = (1.0, 0.86, 0.55)
        rgb[cs.cost[k] == cs.HARD] = (0.62, 0.65, 0.70)
        rgb[pl["occ"]] = (0.25, 0.27, 0.30)
        ax.imshow(rgb, origin="lower", extent=extent(g), interpolation="nearest")
        pose = (pl["pose"][0] + 3.0, 0.0, math.radians(deg))
        outline(ax, ego_poly(pose, 0.0), color=C["car"], lw=1.4)
        ax.plot(pose[0], pose[1], "o", color=C["car"], ms=4)
        ax.set_title("heading %d deg: rear-axle positions in collision (grey), within 0.30 m of it (orange), free (white). "
                     "Blocked cells dark, footprint drawn for scale" % deg)
        ax.set_xlim(pl["pose"][0] - 16, pl["pose"][0] + 16)
        ax.set_ylim(g.y0, g.y0 + g.ny * g.RES)
        ax.set_ylabel("y [m]")
    axes[-1].set_xlabel("x [m]")
    save(fig, "cspace.png")
    return cs


def fig_heuristic(rec):
    sim, pl = rec["sim"], rec["plan"]
    g = sim.grid
    tab = sim.planner.table()
    n = int(round(sim.planner.TAB_RANGE / sim.planner.TAB_RES))
    ax_m = np.arange(-n, n + 1) * sim.planner.TAB_RES
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
    for ax, k, name in ((axes[0], 0, "same heading"), (axes[1], sim.planner.TAB_NPHI // 4, "goal heading +90 deg")):
        im = ax.contourf(ax_m, ax_m, tab[:, :, k].T, levels=np.arange(0, 46, 2.5), cmap="viridis")
        ax.contour(ax_m, ax_m, tab[:, :, k].T, levels=np.arange(0, 46, 5), colors="w", linewidths=0.4)
        ax.plot(0, 0, "w^", ms=7)
        ax.set_title("Reeds-Shepp length to a goal at (x, y), %s [m]" % name)
        ax.set_xlabel("goal x in the car frame [m]")
        ax.set_ylabel("goal y in the car frame [m]")
        ax.set_aspect("equal")
        ax.set_xlim(-15, 15)
        ax.set_ylim(-15, 15)
    fig.colorbar(im, ax=axes[:2], shrink=0.8)
    dist, cell = holonomic_distance(pl["occ"], g.RES, pl["goal"][:2], g.x0, g.y0)
    ax = axes[2]
    d = np.where(np.isfinite(dist), dist, np.nan)
    im2 = ax.imshow(d, origin="lower", extent=(g.x0, g.x0 + dist.shape[1] * cell, g.y0, g.y0 + dist.shape[0] * cell), cmap="magma_r")
    ax.plot(pl["goal"][0], pl["goal"][1], "c*", ms=10)
    ax.set_xlim(pl["pose"][0] - 16, pl["pose"][0] + 16)
    ax.set_title("obstacle-aware 2D distance to the goal [m]")
    ax.set_xlabel("x [m]")
    fig.colorbar(im2, ax=ax, shrink=0.8)
    save(fig, "heuristic.png")


def fig_reeds_shepp():
    fig, axes = plt.subplots(1, 4, figsize=(15, 4.2))
    R = EGO.radius
    for ax, goal in zip(axes, ((9.0, 4.0, 0.0), (0.0, 5.0, math.pi), (-6.0, -6.0, math.pi / 2), (2.0, 3.0, -math.pi / 2))):
        cands = sorted(rs_paths(goal[0] / R, goal[1] / R, goal[2]), key=lambda c: sum(abs(l) for l in c[1]))
        for rank, (word, lens) in enumerate(cands[:3][::-1]):
            rows = rs_sample((0.0, 0.0, 0.0), word, lens, R, 0.1)
            best = rank == 2
            start = np.array([[0.0, 0.0]])
            pts = np.vstack([start, rows[:, :2]])
            dirs = np.concatenate([[rows[0, 3]], rows[:, 3]])
            for d, col in ((1, C["fwd"]), (-1, C["rev"])):
                m = np.ma.masked_where(np.repeat((dirs != d)[:, None], 2, axis=1), pts)
                ax.plot(m[:, 0], m[:, 1], color=col, lw=2.4 if best else 0.8, alpha=1.0 if best else 0.45)
        w, lens = cands[0]
        for pose, col in (((0.0, 0.0, 0.0), "k"), (goal, C["free"])):
            outline(ax, ego_poly(pose), color=col, lw=1.2)
        ax.set_title("shortest word %s, %.1f m" % (w, sum(abs(l) for l in lens) * R))
        ax.set_aspect("equal")
        ax.set_xlabel("x [m]")
    axes[0].set_ylabel("y [m]")
    fig.suptitle("Reeds-Shepp curves for the car's turning radius (%.2f m): forward blue, reverse magenta, two runners-up thin" % R, y=1.02)
    save(fig, "reeds_shepp.png")


def fig_docking():
    """The straight / arc / straight analytic expansion, built from the same formulas as Planner._arc_shot."""
    fig, ax = plt.subplots(figsize=(8.6, 6.2))
    goal, sg = (0.0, 0.0, math.pi / 2), -1                     # back in: the car faces +y and reverses towards -y
    node = (6.2, 9.4, math.radians(28.0))
    gx, gy, gth = goal
    c, s = math.cos(gth), math.sin(gth)
    ly = -(node[0] - gx) * s + (node[1] - gy) * c
    lth = wrap(node[2] - gth)
    k = -sg * math.copysign(EGO.kappa, lth)
    lead = ((1.0 - math.cos(lth)) / k - ly) / (sg * math.sin(lth))
    s_arc = -lth / k
    p1 = (node[0] + sg * lead * math.cos(node[2]), node[1] + sg * lead * math.sin(node[2]))
    sa = np.linspace(0.0, s_arc, 60)
    th = node[2] + k * sa
    arc = np.stack([p1[0] + (np.sin(th) - math.sin(node[2])) / k, p1[1] - (np.cos(th) - math.cos(node[2])) / k], axis=1)
    ax.plot([node[0], p1[0]], [node[1], p1[1]], color=C["rev"], lw=2.5, label="lead-in straight")
    ax.plot(arc[:, 0], arc[:, 1], color="#7a3fd0", lw=2.5, label="one arc, curvature k")
    ax.plot([arc[-1, 0], gx], [arc[-1, 1], gy], color=C["free"], lw=2.5, label="docking run along the stall axis")
    ax.plot([gx, gx], [gy - 2.0, gy + 13.5], color=C["grey"], ls=":", lw=1.0)
    ctr = (p1[0] - math.sin(node[2]) / k, p1[1] + math.cos(node[2]) / k)
    ax.plot(*ctr, "+", color="#7a3fd0", ms=10)
    ax.plot([ctr[0], p1[0]], [ctr[1], p1[1]], color="#7a3fd0", lw=0.6, ls="--")
    ax.plot([ctr[0], arc[-1, 0]], [ctr[1], arc[-1, 1]], color="#7a3fd0", lw=0.6, ls="--")
    for pose, col, name in ((node, "k", "search node"), ((p1[0], p1[1], node[2]), C["rev"], ""), ((arc[-1, 0], arc[-1, 1], gth), "#7a3fd0", ""),
                            (goal, C["free"], "parked pose")):
        outline(ax, ego_poly(pose), color=col, lw=1.1)
        ax.plot(pose[0], pose[1], "o", color=col, ms=3.5)
        if name:
            ax.annotate(name, (pose[0], pose[1]), textcoords="offset points", xytext=(8, -12), color=col)
    ax.annotate("stall axis", (gx, gy + 12.8), textcoords="offset points", xytext=(6, 0), color=C["grey"])
    ax.annotate("lead = %.2f m" % lead, (0.5 * (node[0] + p1[0]), 0.5 * (node[1] + p1[1])), textcoords="offset points", xytext=(8, -4), color=C["rev"])
    ax.annotate("arc: turns by %.0f deg, R = %.2f m" % (abs(math.degrees(lth)), 1 / abs(k)), arc[30], textcoords="offset points", xytext=(10, 0), color="#7a3fd0")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.legend(loc="lower right", frameon=False)
    ax.set_title("analytic docking expansion: straight, one arc onto the stall axis, straight (all in reverse here)")
    save(fig, "docking_shot.png")


def fig_plans(recs):
    fig, axes = plt.subplots(3, 1, figsize=(12, 12.5), gridspec_kw=dict(height_ratios=[1.0, 1.0, 0.62]))
    for ax, rec in zip(axes, recs):
        sim, pl = rec["sim"], rec["plan"]
        draw_scene(ax, sim.scn)
        e = pl["explored"]
        ax.plot(e[:, 0], e[:, 1], ".", color="#5aaac2", ms=1.6, alpha=0.7, zorder=3)
        for x, y, th, d in pl["segs"]:
            ax.plot(x, y, color=C["fwd"] if d > 0 else C["rev"], lw=2.6, zorder=6)
        pose = np.array(rec["pose"])
        drive = np.array([s in ("DRIVE", "BRAKE", "PARKED") for s in rec["state"]])
        ax.plot(pose[drive, 0], pose[drive, 1], "k-", lw=0.8, zorder=7)
        idx = np.flatnonzero(drive)
        for i in list(idx[::40]) + [idx[-1]]:
            outline(ax, ego_poly(pose[i]), color=C["car"], lw=0.5 if i != idx[-1] else 1.8, zorder=5)
        st = pl["stats"]
        ax.set_title("%s: %s   (%d expansions, cost %.1f, margin %.2f m)\nsearch tree in teal, planned path in colour, driven path black" % (
            sim.scn.name, " + ".join("%s %.1f m" % ("fwd" if d > 0 else "rev", np.hypot(np.diff(x), np.diff(y)).sum()) for x, y, th, d in pl["segs"]),
            st["iterations"], st["cost"], st["margin"]))
        ax.set_xlim(pl["pose"][0] - 15, pl["pose"][0] + 17)
        g = sim.grid
        ax.set_ylim(g.y0, g.y0 + g.ny * g.RES)
    fig.subplots_adjust(hspace=0.32)
    save(fig, "plans.png")


def fig_mpc(rec):
    m = rec["mpc"]
    mpc = LateralMPC()
    N, h = mpc.N, mpc.H
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.3))
    k_now, k_max = m["g"] * math.tan(m["steer"]), m["g"] * math.tan(EGO.steer_max)
    dk = min(m["g"] * (1.0 + math.tan(m["steer"]) ** 2) * STEER_RATE * h / max(abs(m["v"]), 0.3), 2.0 * k_max)
    K, pred = mpc.solve(m["d"], m["e"], m["psi"], k_now, m["k_ref"], k_max, dk)
    Ku, predu = LateralMPC().solve(m["d"], m["e"], m["psi"], k_now, m["k_ref"], 10.0, 10.0)
    s = (np.arange(N) + 0.5) * h
    ax = axes[0]
    ax.step(s, m["k_ref"], where="mid", color=C["grey"], lw=1.5, label="path curvature (reference)")
    ax.step(s, Ku, where="mid", color="#9a6fd0", lw=1.0, ls="--", label="unconstrained optimum")
    ax.step(s, K, where="mid", color=C["mpc"], lw=2.2, label="MPC plan")
    ax.axhline(k_max, color=C["occ"], lw=0.8, ls=":")
    ax.axhline(-k_max, color=C["occ"], lw=0.8, ls=":", label="steering limit  g tan(delta_max)")
    ax.plot(0, k_now, "ko", ms=4, label="curvature now")
    ax.set_xlabel("distance ahead [m]")
    ax.set_ylabel("curvature [1/m]")
    ax.set_title("curvature over the horizon (rate limit %.3f per step)" % dk)
    ax.legend(frameon=False, fontsize=8)
    ax = axes[1]
    sk = np.arange(N + 1) * h
    ax.plot(sk, 100 * np.concatenate([[m["e"]], pred[:, 0]]), color=C["track"], lw=2, label="lateral error e [cm]")
    ax.plot(sk, np.degrees(np.concatenate([[m["psi"]], pred[:, 1]])), color="#e07a10", lw=2, label="heading error psi [deg]")
    ax.plot(sk, 100 * np.concatenate([[m["e"]], predu[:, 0]]), color=C["track"], lw=0.9, ls="--")
    ax.plot(sk, np.degrees(np.concatenate([[m["psi"]], predu[:, 1]])), color="#e07a10", lw=0.9, ls="--")
    ax.axhline(0, color="k", lw=0.5)
    ax.set_xlabel("distance ahead [m]")
    ax.set_title("predicted errors (dashed: if the steering had no limits)")
    ax.legend(frameon=False, fontsize=8)
    ax = axes[2]
    ax.plot(m["seg"][0], m["seg"][1], color=C["rev"] if m["d"] < 0 else C["fwd"], lw=2.0, label="planned path")
    ax.plot(m["horizon"][:, 0], m["horizon"][:, 1], "o-", color=C["mpc"], ms=3, lw=1.2, label="MPC prediction (rear axle)")
    outline(ax, ego_poly(m["pose"]), color=C["car"], lw=1.3)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("the same solve in the world (%s, %.2f m/s)" % ("reverse" if m["d"] < 0 else "forward", abs(m["v"])))
    ax.legend(frameon=False, fontsize=8)
    save(fig, "mpc_solve.png")


def fig_tracking(rec, name):
    t = rec["t"]
    state = np.array(rec["state"])
    fig, axes = plt.subplots(5, 1, figsize=(12, 11), sharex=True)
    spans = []
    start = 0
    for i in range(1, len(state) + 1):
        if i == len(state) or state[i] != state[start]:
            spans.append((t[start], t[i - 1], state[start]))
            start = i
    shade = dict(SEARCH="#e8f1ff", BRAKE="#fff3d6", DRIVE="#e9f8ec", PARKED="#f0f0f0")
    for ax in axes:
        for a, b, st in spans:
            if st in shade:
                ax.axvspan(a, b, color=shade[st], lw=0)
        ax.grid(True, lw=0.3)
    for a, b, st in spans:
        if st in shade and b - a > 1.5:
            axes[0].text(0.5 * (a + b), 2.45, st.lower(), ha="center", va="top", fontsize=8, color="#555")
    axes[0].plot(t, rec["vcmd"], color=C["grey"], lw=1.2, label="speed command")
    axes[0].plot(t, rec["v"], color="#1c9c4a", lw=1.6, label="speed")
    axes[0].set_ylabel("m/s")
    axes[0].set_ylim(-1.6, 2.6)
    axes[0].legend(frameon=False, ncol=2, loc="lower left")
    axes[1].plot(t, rec["drive"], color="#1c9c4a", lw=1.4, label="drive torque at the wheels (negative = reverse)")
    axes[1].plot(t, -rec["brake"], color=C["occ"], lw=1.4, label="brake torque (drawn negative)")
    axes[1].set_ylabel("N m")
    axes[1].legend(frameon=False, ncol=2, loc="lower left")
    axes[2].plot(t, np.degrees(rec["steer"]), color=C["fwd"], lw=1.4)
    axes[2].axhline(math.degrees(EGO.steer_max), color=C["occ"], lw=0.8, ls=":")
    axes[2].axhline(-math.degrees(EGO.steer_max), color=C["occ"], lw=0.8, ls=":")
    axes[2].set_ylabel("road-wheel angle [deg]")
    axes[2].set_ylim(-40, 40)
    axes[3].plot(t, 100 * rec["e"], color=C["track"], lw=1.4, label="lateral error [cm]")
    axes[3].plot(t, np.degrees(rec["psi"]), color="#e07a10", lw=1.4, label="heading error [deg]")
    axes[3].set_ylim(-9, 9)
    axes[3].legend(frameon=False, ncol=2, loc="lower left")
    axes[4].axhline(1.0 / EGO.wheelbase, color=C["grey"], lw=1.0, ls="--", label="ideal bicycle, 1 / wheelbase")
    axes[4].plot(t, rec["gf"], color=C["fwd"], lw=1.6, label="identified gain, forward")
    axes[4].plot(t, rec["gr"], color=C["rev"], lw=1.6, label="identified gain, reverse")
    axes[4].set_ylabel("curvature / tan(angle) [1/m]")
    axes[4].set_xlabel("time [s]")
    axes[4].set_ylim(0.15, 0.50)
    axes[4].legend(frameon=False, ncol=3, loc="lower left")
    axes[0].set_title("%s: signals of one run (background: search, stopping, driving the plan, parked)" % rec["sim"].scn.name)
    save(fig, name)


if __name__ == "__main__":
    perp = run("perpendicular", "both", 3)
    angled = run("angled", "both", 1)
    par = run("parallel", "both", 1)
    fig_scenarios()
    fig_perception(perp)
    fig_mapping(perp)
    fig_cspace(perp)
    fig_heuristic(perp)
    fig_reeds_shepp()
    fig_docking()
    fig_plans([perp, angled, par])
    fig_mpc(perp if perp["mpc"] is not None else par)
    fig_tracking(perp, "tracking_perpendicular.png")
    fig_tracking(par, "tracking_parallel.png")
