#!/usr/bin/env python3
"""Regenerates the sensor figures in docs/img from real runs with Chrono::Sensor.

Recording needs a PyChrono that has the ray-traced sensors. Drawing needs matplotlib. If one
Python has both:

    python docs/make_sensor_figures.py

Otherwise in two steps, with the Python that has each:

    python docs/make_sensor_figures.py record frames.npz
    python docs/make_sensor_figures.py draw frames.npz

Three headless simulations are run one after another (camera, camera + lidar, camera + radar)."""

import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, "img")


def load_sim():
    spec = importlib.util.spec_from_file_location("parking_sim", os.path.join(HERE, "..", "parking_sim.py"))
    ps = importlib.util.module_from_spec(spec)
    sys.modules["parking_sim"] = ps
    spec.loader.exec_module(ps)
    return ps


# ------------------------------------------------------------------------------------------------
# Recording
# ------------------------------------------------------------------------------------------------

def record(path):
    """Each sensor set runs in a process of its own: a ray-traced scene holds a few GB."""
    import subprocess
    out = {}
    for mode in ("camera", "camera+lidar", "camera+radar"):
        part = "%s.%s.npz" % (path, mode.replace("+", "_"))
        if subprocess.call([sys.executable, os.path.abspath(__file__), "record-one", mode, part]) != 0:
            sys.exit("recording %s failed" % mode)
        with np.load(part) as d:
            out.update({k: d[k] for k in d.files})
        os.remove(part)
    np.savez_compressed(path, **out)
    print("wrote " + path, flush=True)


def record_one(mode, path):
    ps = load_sim()
    if not ps.HAVE_SENSORS:
        sys.exit("this PyChrono has no ray-traced sensors, see docs/sensors.md")
    out = {}
    if True:
        args = ps.parse_args(["--headless", "--sensors", mode, "--type", "perpendicular", "--cars", "both"])
        sim = ps.ParkingSim(args)
        rig, tag = sim.sensor, mode.replace("camera", "c").replace("+lidar", "l").replace("+radar", "r")
        frame = {}
        inner = rig._camera

        def spy(cam, rgb, depth, ref_p, ref_R, inner=inner, frame=frame, sim=sim):
            res = inner(cam, rgb, depth, ref_p, ref_R)
            # the frame the camera figure is made of: the front camera, a stall coming into view
            if not frame and cam["label"] == "front" and sim.state == "SEARCH" and sim.pose[0] > 3.0:
                scan, xy, segs = res
                frame.update(rgb=np.array(rgb[..., :3]), depth=np.array(depth), view=cam["view"].copy(),
                             origin=np.array(scan[0]), ang=scan[1], r_hit=scan[2], r_free=scan[3], r_far=scan[4],
                             paint=xy, segs=np.array(segs).reshape(-1, 5), pose=np.array(sim.pose))
            return res

        rig._camera = spy
        plan = None
        while sim.result is None and sim.time < 120.0:
            sim.advance(50)
            if plan is None and sim.plan_info is not None:
                g = sim.grid
                plan = dict(free=g.free.copy(), far=g.far.copy(), stop=g.stop.copy(), occ=g.occupied().copy(),
                            blocked=g.blocked().copy(), pose=np.array(sim.pose),
                            path=np.concatenate([np.stack([s.x, s.y], axis=1) for s in sim.path]),
                            tracks=np.array([np.concatenate(t.ends()) for t in sim.lines.markers()]).reshape(-1, 4),
                            long=np.array([t.length > 1.2 for t in sim.lines.markers()]),
                            target=sim.target.corners.copy() if sim.target is not None else np.zeros((0, 2)),
                            fans=np.array([[f[0][0], f[0][1], f[1], f[2], f[3]] for f in rig.fans]))
        print("%s: %s" % (mode, sim.result), flush=True)
        for k, v in plan.items():
            out["%s_plan_%s" % (tag, k)] = v
        if mode == "camera":
            for k, v in frame.items():
                out["frame_" + k] = v
            E = ps.EGO
            out["ego"] = np.array([E.rear, E.front, E.half_width, E.ref_to_rear, *E.roof])
            out["mounts"] = np.array([np.append(c["pos"], math.atan2(c["R"][1, 0], c["R"][0, 0])) for c in rig.cameras])
            out["extent"] = np.array([sim.grid.x0, sim.grid.x0 + sim.grid.nx * sim.grid.RES,
                                      sim.grid.y0, sim.grid.y0 + sim.grid.ny * sim.grid.RES])
            out["lines"] = np.array([l[:4] for l in sim.scn.lines])
            out["cars"] = np.array([c["poly"] for c in sim.scn.cars])
            out["cam"] = np.array([rig.CAM_W, rig.CAM_H, rig.CAM_HFOV, rig.CAM_PITCH, rig.CAM_RANGE])
        elif mode == "camera+lidar":
            out["lidar"] = np.append(rig.lidar["pos"], [*rig.LIDAR_EL, rig.LIDAR_RANGE])
        else:
            out["radars"] = np.array([np.append(r["pos"], math.atan2(r["R"][1, 0], r["R"][0, 0])) for r in rig.radars])
            out["radar"] = np.array([rig.RADAR_HFOV, rig.RADAR_VFOV, rig.RADAR_RANGE])
    np.savez_compressed(path, **out)
    sys.stdout.flush()
    os._exit(0)             # leave without running destructors: the sensor manager owns render threads


# ------------------------------------------------------------------------------------------------
# Drawing
# ------------------------------------------------------------------------------------------------

def draw(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap
    from matplotlib.patches import Polygon, Wedge

    d = np.load(path)
    os.makedirs(IMG, exist_ok=True)
    plt.rcParams.update({"font.size": 9, "axes.titlesize": 10, "figure.dpi": 110, "savefig.bbox": "tight"})
    C = dict(cam="#2f6fe0", lidar="#22a745", radar="#d9412b", car="#b02a2a", paint="#e0a800", seg="#d63ec8",
             hit="#111111", free="#dfe9df", far="#f3ead0", unknown="#6b7078", occ="#111111", line="#c9a400",
             track="#0aa5c4", stub="#0a6f82", path="#d63ec8", target="#22a745")

    def save(fig, name):
        fig.savefig(os.path.join(IMG, name), dpi=130)
        plt.close(fig)
        print("wrote docs/img/" + name)

    rear, front, hw, ref_to_rear, roof_x0, roof_x1, roof_z = d["ego"]

    # ---- the rig: where the sensors sit and what they cover -----------------------------------
    fig, axs = plt.subplots(2, 1, figsize=(8.0, 10.2), gridspec_kw=dict(height_ratios=[2.6, 1.0]))
    ax = axs[0]
    x0, x1 = -ref_to_rear - rear, -ref_to_rear + front           # body outline in the chassis frame
    ax.add_patch(Polygon([(x0, -hw), (x1, -hw), (x1, hw), (x0, hw)], closed=True, fc="#e7c1c1", ec=C["car"], lw=1.5))
    ax.add_patch(Polygon([(roof_x0, -0.6), (roof_x1, -0.6), (roof_x1, 0.6), (roof_x0, 0.6)], closed=True,
                         fc="#d79a9a", ec=C["car"], lw=0.8))
    hfov, reach = float(d["cam"][2]), float(d["cam"][4])
    for (mx, my, mz, yaw), name in zip(d["mounts"], ("front stereo camera", "rear stereo camera")):
        ax.add_patch(Wedge((mx, my), reach, math.degrees(yaw - 0.5 * hfov), math.degrees(yaw + 0.5 * hfov),
                           fc=C["cam"], alpha=0.13, ec=C["cam"]))
        ax.plot(mx, my, "s", color=C["cam"], ms=6)
        ax.annotate(name, (mx + 4.5 * math.cos(yaw), my + 3.2), color=C["cam"], ha="center", fontsize=8)
    lx, ly, lz, el0, el1, lr = d["lidar"]
    ax.add_patch(plt.Circle((lx, ly), 14.0, fc="none", ec=C["lidar"], ls="--", lw=1.0))
    ax.plot(lx, ly, "o", color=C["lidar"], ms=6)
    ax.annotate("lidar, 360 deg\n(camera + lidar)", (lx, -13.2), color=C["lidar"], ha="center", fontsize=8)
    r_hfov = float(d["radar"][0])
    for mx, my, mz, yaw in d["radars"]:
        ax.add_patch(Wedge((mx, my), 9.0, math.degrees(yaw - 0.5 * r_hfov), math.degrees(yaw + 0.5 * r_hfov),
                           fc=C["radar"], alpha=0.10, ec=C["radar"]))
        ax.plot(mx, my, "^", color=C["radar"], ms=6)
    ax.annotate("side radars\n(camera + radar)", (0.0, 9.6), color=C["radar"], ha="center", fontsize=8)
    ax.set_aspect("equal")
    ax.set_xlim(-15.5, 15.5)
    ax.set_ylim(-15, 15)
    ax.set_xlabel("x [m], forward")
    ax.set_ylabel("y [m], left")
    ax.set_title("From above. No camera looks sideways.")
    ax = axs[1]
    prof_x = [x0, x0, roof_x0 - 0.6, roof_x0, roof_x1, roof_x1 + 0.75, x1, x1]
    prof_z = [0.0, 0.95, 1.0, roof_z, roof_z, 0.95, 0.75, 0.0]
    ground = -0.21                                              # the chassis frame sits above the ground
    ax.add_patch(Polygon(list(zip(prof_x, prof_z)), closed=True, fc="#e7c1c1", ec=C["car"], lw=1.5))
    ax.axhline(ground, color="0.4", lw=1.0)
    pitch = float(d["cam"][3])
    vfov = 2.0 * math.atan(math.tan(0.5 * hfov) * d["cam"][1] / d["cam"][0])
    for (mx, my, mz, yaw), (ex, ez) in zip(d["mounts"], ((x1, 0.75), (x0, 0.95))):
        s = math.cos(yaw)
        up = -pitch + 0.5 * vfov                                 # upper edge of the image
        ax.plot([mx, mx + s * 4.5 * math.cos(up)], [mz, mz + 4.5 * math.sin(up)], color=C["cam"], lw=0.9)
        gx = mx + (ex - mx) * (mz - ground) / (mz - ez)          # the ray that grazes the end of the body
        ax.plot([mx, gx], [mz, ground], color=C["cam"], lw=0.9)
        ax.fill([mx, mx + s * 4.5 * math.cos(up), gx], [mz, mz + 4.5 * math.sin(up), ground], color=C["cam"], alpha=0.08)
        ax.annotate("first ground seen\n%.1f m from the bumper" % abs(gx - ex), (gx, ground), (gx + s * 0.4, 1.9),
                    color=C["cam"], fontsize=7, ha="center", arrowprops=dict(arrowstyle="-", color=C["cam"], lw=0.6))
        ax.plot(mx, mz, "s", color=C["cam"], ms=6)
    for a in np.linspace(el0, el1, 16):
        t = min(8.5, (lz - ground) / max(-math.sin(a), 1e-6)) if a < 0 else 8.5
        for s in (-1.0, 1.0):
            ax.plot([lx, lx + s * t * math.cos(a)], [lz, lz + t * math.sin(a)], color=C["lidar"], lw=0.35)
    ax.plot(lx, lz, "o", color=C["lidar"], ms=6)
    ax.plot(d["radars"][0][0], d["radars"][0][2], "^", color=C["radar"], ms=6)
    ax.set_aspect("equal")
    ax.set_xlim(-9.5, 9.5)
    ax.set_ylim(-0.6, 3.4)
    ax.set_xlabel("x [m], forward")
    ax.set_ylabel("z [m], chassis frame")
    ax.set_title("From the side. The bonnet and the boot hide the nearest ground from the cameras.")
    fig.tight_layout()
    save(fig, "sensor_rig.png")

    # ---- one camera frame: what comes in and what is made of it -----------------------------------
    fig = plt.figure(figsize=(13.0, 6.4))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.0, 1.0, 1.05], height_ratios=[1.0, 1.0])
    ax = fig.add_subplot(gs[0, 0])
    ax.imshow(d["frame_rgb"][::-1])
    ax.set_title("colour image, front camera (%d x %d, %.0f deg)" % (d["cam"][0], d["cam"][1], math.degrees(hfov)))
    ax.axis("off")
    ax = fig.add_subplot(gs[0, 1])
    im = ax.imshow(np.minimum(d["frame_depth"][::-1], 20.0), cmap="viridis_r")
    ax.set_title("depth image: range along each pixel's ray")
    ax.axis("off")
    fig.colorbar(im, ax=ax, fraction=0.03, pad=0.02, label="m")
    ax = fig.add_subplot(gs[1, 0])
    cmap = ListedColormap(["#222630", "#54585f", "#ff7828", "#702c2c", "#96784a", "#fff03c"])
    ax.imshow(d["frame_view"], cmap=cmap, vmin=0, vmax=5, interpolation="nearest")
    ax.set_title("what each pixel is read as")
    ax.axis("off")
    ax = fig.add_subplot(gs[1, 1])
    ax.axis("off")
    for k, (name, col) in enumerate((("ground", "#54585f"), ("obstacle", "#ff7828"), ("unclear: not ground, not clearly an obstacle", "#96784a"),
                                     ("paint", "#fff03c"), ("the car itself", "#702c2c"), ("nothing within range, or sky", "#222630"))):
        ax.add_patch(plt.Rectangle((0.02, 0.86 - 0.13 * k), 0.07, 0.085, fc=col, ec="0.3", transform=ax.transAxes))
        ax.text(0.12, 0.90 - 0.13 * k, name, transform=ax.transAxes, va="center", fontsize=9)
    ax = fig.add_subplot(gs[:, 2])
    for x1_, y1_, x2_, y2_ in d["lines"]:
        ax.plot([x1_, x2_], [y1_, y2_], color="#d8d8d8", lw=4, solid_capstyle="butt", zorder=1)
    for poly in d["cars"]:
        ax.add_patch(Polygon(poly, closed=True, fc="#eceef1", ec="#9aa0a8", lw=0.8, zorder=2))
    o = d["frame_origin"]
    for a, rf, rr in zip(d["frame_ang"][::3], d["frame_r_free"][::3], d["frame_r_far"][::3]):
        ax.plot([o[0] + rf * math.cos(a), o[0] + rr * math.cos(a)], [o[1] + rf * math.sin(a), o[1] + rr * math.sin(a)],
                color="#c9b26b", lw=0.5, zorder=3)
        ax.plot([o[0], o[0] + rf * math.cos(a)], [o[1], o[1] + rf * math.sin(a)], color="#6fae6f", lw=0.5, zorder=3)
    ok = np.isfinite(d["frame_r_hit"])
    ax.plot(o[0] + d["frame_r_hit"][ok] * np.cos(d["frame_ang"][ok]), o[1] + d["frame_r_hit"][ok] * np.sin(d["frame_ang"][ok]),
            ".", color=C["hit"], ms=3, zorder=5, label="nearest obstacle per bearing")
    ax.plot(d["frame_paint"][::2, 0], d["frame_paint"][::2, 1], ",", color=C["paint"], zorder=4)
    for k, s in enumerate(d["frame_segs"]):
        ax.plot([s[0], s[2]], [s[1], s[3]], color=C["seg"], lw=2.0, zorder=6, label="line segments" if k == 0 else None)
    ax.plot([], [], color="#6fae6f", lw=1.5, label="free ground")
    ax.plot([], [], color="#c9b26b", lw=1.5, label="ground, too far to rule out a kerb")
    ax.plot([], [], "s", color=C["paint"], ms=4, label="paint pixels on the ground")
    px, py, pth = d["frame_pose"]
    c, s_ = math.cos(pth), math.sin(pth)
    body = [(px + c * a - s_ * b, py + s_ * a + c * b) for a, b in ((-rear, -hw), (front, -hw), (front, hw), (-rear, hw))]
    ax.add_patch(Polygon(body, closed=True, fc="#e7c1c1", ec=C["car"], lw=1.2, zorder=7))
    ax.set_aspect("equal")
    ax.set_xlim(px - 3.0, px + 14.5)
    ax.set_ylim(-12.5, 12.5)
    ax.legend(loc="upper left", fontsize=7, framealpha=0.95)
    ax.set_title("the same frame on the ground: scan and line segments")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.tight_layout()
    save(fig, "camera_frame.png")

    # ---- what each sensor set knows when the plan is made ---------------------------------------
    names = (("c", "camera"), ("cl", "camera + lidar"), ("cr", "camera + radar"))
    fig, axs = plt.subplots(3, 1, figsize=(11.5, 11.0))
    ext = d["extent"]
    for ax, (tag, name) in zip(axs, names):
        free, far, stop, occ = (d["%s_plan_%s" % (tag, k)] for k in ("free", "far", "stop", "occ"))
        img = np.zeros(free.shape, dtype=np.uint8)                 # 0 unknown
        img[(far >= 3) & (stop == 0)] = 1                          # far ground
        img[free >= 1] = 2                                         # free
        img[occ] = 3
        ax.imshow(img, origin="lower", extent=ext, interpolation="nearest",
                  cmap=ListedColormap([C["unknown"], C["far"], C["free"], C["occ"]]), vmin=0, vmax=3)
        for x1_, y1_, x2_, y2_ in d["lines"]:
            ax.plot([x1_, x2_], [y1_, y2_], color=C["line"], lw=2.5, alpha=0.35, solid_capstyle="butt")
        for poly in d["cars"]:
            ax.add_patch(Polygon(poly, closed=True, fc="none", ec="#4c7fd0", lw=0.7))
        for (ax1, ay1, ax2, ay2), lng in zip(d[tag + "_plan_tracks"], d[tag + "_plan_long"]):
            ax.plot([ax1, ax2], [ay1, ay2], color=C["track"] if lng else C["stub"], lw=2.2 if lng else 3.2)
        tgt = d[tag + "_plan_target"]
        if len(tgt):
            ax.add_patch(Polygon(tgt, closed=True, fc="none", ec=C["target"], lw=2.2))
        p = d[tag + "_plan_path"]
        ax.plot(p[:, 0], p[:, 1], color=C["path"], lw=1.6)
        px, py, pth = d[tag + "_plan_pose"]
        c, s_ = math.cos(pth), math.sin(pth)
        body = [(px + c * a - s_ * b, py + s_ * a + c * b) for a, b in ((-rear, -hw), (front, -hw), (front, hw), (-rear, hw))]
        ax.add_patch(Polygon(body, closed=True, fc="#e7c1c1", ec=C["car"], lw=1.2))
        ax.set_xlim(-12.0, ext[1])
        ax.set_ylim(ext[2], ext[3])
        ax.set_aspect("equal")
        ax.set_ylabel("y [m]")
        ax.set_title("%s: the map when the plan is made. Free (light green), far ground (sand), obstacle (black), "
                     "unknown (grey)" % name)
    axs[-1].set_xlabel("x [m]")
    fig.tight_layout()
    save(fig, "sensor_maps.png")


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "record-one":
        record_one(sys.argv[2], sys.argv[3])
    elif len(sys.argv) == 3 and sys.argv[1] in ("record", "draw"):
        (record if sys.argv[1] == "record" else draw)(sys.argv[2])
    elif len(sys.argv) == 1:
        import tempfile
        tmp = os.path.join(tempfile.gettempdir(), "parking_sensor_frames.npz")
        record(tmp)
        draw(tmp)
    else:
        sys.exit(__doc__)
