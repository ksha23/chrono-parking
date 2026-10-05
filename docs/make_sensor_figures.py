#!/usr/bin/env python3
"""Regenerates the sensor figures in docs/img from real runs with Chrono::Sensor, and measures
how good the perception is against the geometry of the scenario.

Recording needs a PyChrono that has the ray-traced sensors, and the depth networks set up (see
docs/sensors.md). Drawing needs matplotlib. If one Python has both:

    python docs/make_sensor_figures.py

Otherwise in two steps, with the Python that has each:

    python docs/make_sensor_figures.py record frames.pkl
    python docs/make_sensor_figures.py draw frames.pkl

Two headless simulations are run one after another (camera, camera + lidar).

The reference for the accuracy figures is the scenario itself, not anything rendered: the road is
a plane, so the true range of every pixel that shows the road follows from where the camera is,
and the true range to the nearest obstacle on a bearing follows from the outlines of the parked
cars and the kerbs."""

import importlib.util
import math
import os
import pickle
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, "img")
SNAPSHOTS = (9.0, 27.0)          # simulated times at which the camera frames are kept [s]
KIND = {"front": 0, "rear": 1, "bumper": 2}


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
    for mode in ("camera", "camera+lidar"):
        part = "%s.%s.pkl" % (path, mode.replace("+", "_"))
        if subprocess.call([sys.executable, os.path.abspath(__file__), "record-one", mode, part]) != 0:
            sys.exit("recording %s failed" % mode)
        out[mode] = pickle.load(open(part, "rb"))
        os.remove(part)
    pickle.dump(out, open(path, "wb"))
    print("wrote " + path, flush=True)


def first_hit(origin, ang, edges, r_max):
    """Range from a point to the nearest of the edges (x1, y1, x2, y2) along each bearing."""
    d = np.stack([np.cos(ang), np.sin(ang)], axis=1)[:, None, :]
    a, e = edges[None, :, :2] - np.asarray(origin)[None, None, :], (edges[:, 2:] - edges[:, :2])[None]
    den = d[..., 0] * e[..., 1] - d[..., 1] * e[..., 0]
    with np.errstate(divide="ignore", invalid="ignore"):
        t = (a[..., 0] * e[..., 1] - a[..., 1] * e[..., 0]) / den
        s = (a[..., 0] * d[..., 1] - a[..., 1] * d[..., 0]) / den
    t = np.where((np.abs(den) > 1e-9) & (t > 0.0) & (s >= 0.0) & (s <= 1.0), t, np.inf)
    return np.minimum(t.min(axis=1), r_max)


def record_one(mode, path):
    ps = load_sim()
    if not ps.HAVE_SENSORS:
        sys.exit("this PyChrono has no ray-traced sensors, see docs/sensors.md")
    args = ps.parse_args(["--headless", "--sensors", mode, "--type", "perpendicular", "--cars", "both"])
    sim = ps.ParkingSim(args)
    rig, scn = sim.sensor, sim.scn
    rects = [ps.rect_poly(cx, cy, 0.0, -0.5 * lx, 0.5 * lx, 0.5 * ly) for cx, cy, lx, ly, _ in scn.pads]
    polys = scn.obstacle_polys() + rects                   # parked cars, kerbs, and the raised ground beside the lot
    edges = np.array([np.concatenate([p[k], p[(k + 1) % len(p)]]) for p in polys for k in range(len(p))])
    lines = np.array([ln[:4] for ln in scn.lines], dtype=float)
    road, hits, segs, latest, frames = [], [], [], {}, {}
    rng = np.random.default_rng(0)

    def off_line(q):                                       # distance of a point from the nearest painted line
        a, ab = lines[:, :2], lines[:, 2:] - lines[:, :2]
        t = np.clip(((q[None] - a) * ab).sum(1) / (ab * ab).sum(1), 0.0, 1.0)
        return float(np.hypot(*(a + t[:, None] * ab - q[None]).T).min())

    camera = ps.SensorRig._camera

    def camera_checked(self, cam, image, depth, ref_p, ref_R):
        scan, xy, found = camera(self, cam, image, depth, ref_p, ref_R)
        p, R = ref_p + ref_R @ cam["pos"], ref_R @ cam["R"]
        kind = KIND[cam["label"]]
        latest[cam["label"]] = dict(image=image[::-1].copy(), range=depth[::-1].copy(), view=cam["view"].copy())
        # the road as a reference: pixels whose ray reaches the ground with nothing in between
        pick = rng.integers(0, depth.size, 2500)
        dw = (self.rays.reshape(-1, 3)[pick] @ R.T.astype(np.float32)).astype(float)
        est = depth.reshape(-1)[pick].astype(float)
        ok = (dw[:, 2] < -0.02) & (est > 0.3)
        true = np.where(ok, -p[2] / np.minimum(dw[:, 2], -0.02), np.inf)
        ok &= true < 15.0
        g = p[:2] + np.where(ok, true, 0.0)[:, None] * dw[:, :2]
        # (not the pixels that show the car itself: the bonnet hides the road up to 3 m ahead of the bumper)
        seen_at = (p + est[:, None] * dw).astype(np.float32)
        ok &= ~self._is_own(seen_at, ref_p, ref_R, 0.3)
        rho = np.hypot(*(g - p[:2]).T)
        ok[ok] = first_hit(p[:2], np.arctan2(dw[ok, 1], dw[ok, 0]), edges, 40.0) > rho[ok] + 0.3
        road.append(np.stack([np.full(ok.sum(), kind), true[ok], est[ok]], axis=1))
        # the nearest obstacle per bearing against the outlines
        origin, ang, r_hit = scan[0], scan[1], scan[2]
        m = np.isfinite(r_hit)
        if m.any():
            hits.append(np.stack([np.full(m.sum(), kind), first_hit(origin, ang[m], edges, 40.0), r_hit[m]], axis=1))
        for x1, y1, x2, y2, _ in found:
            worst = max(off_line(q) for q in np.linspace((x1, y1), (x2, y2), 7))
            segs.append((kind, math.hypot(x2 - x1, y2 - y1), worst))
        return scan, xy, found

    ps.SensorRig._camera = camera_checked
    scanner = ps.SensorRig._scanner

    def scanner_checked(self, dev, r, *rest):
        out = scanner(self, dev, r, *rest)
        origin, ang, r_hit = out[0], out[1], out[2]
        m = np.isfinite(r_hit)
        if m.any():
            hits.append(np.stack([np.full(m.sum(), 3), first_hit(origin, ang[m], edges, 40.0), r_hit[m]], axis=1))
        return out

    ps.SensorRig._scanner = scanner_checked
    todo = list(SNAPSHOTS)
    trail = []
    while sim.result is None and sim.time < args.timeout:
        sim.advance(50)
        trail.append(sim.pose)
        if todo and sim.time >= todo[0] and len(latest) == 3:
            frames[todo.pop(0)] = {k: dict(v) for k, v in latest.items()}
    g = sim.grid
    # the car from the side: the top and the bottom of its body along the middle, from the mesh
    shape = ps.chrono.CastToChVisualShapeTriangleMesh(rig.body.GetVisualModel().GetShape(0))
    verts = np.array([[float(v) for v in ln.split()[1:4]] for ln in open(shape.GetMesh().GetFileName()) if ln.startswith("v ")])
    mid = verts[np.abs(verts[:, 1]) < 0.3]
    bins = np.round(mid[:, 0] / 0.05).astype(int)
    profile = np.array([(b * 0.05, mid[bins == b, 2].max(), mid[bins == b, 2].min()) for b in np.unique(bins)])
    out = dict(mode=mode, result=sim.result, message=sim.message, road=np.concatenate(road), hits=np.concatenate(hits),
               segs=np.array(segs), frames=frames, free=g.free.copy(), far=g.far.copy(), obstacles=g.hits.copy(),
               grid=(g.x0, g.y0, g.RES, g.nx, g.ny), trail=np.array(trail), parked=ps.ego_poly(sim.pose),
               lines=scn.lines, cars=[c["poly"] for c in scn.cars],
               tracks=[t.ends() for t in sim.lines.markers()],
               cameras=[dict(label=c["label"], pos=c["pos"], R=c["R"], reach=c.get("reach", 0.0)) for c in rig.cameras],
               lidar=None if rig.lidar is None else rig.lidar["pos"], profile=profile, own=rig.own,
               ref_height=float(rig._frame()[0][2]), hfov=rig.CAM_HFOV, vfov=2.0 * math.atan(0.5 * rig.CAM_H / rig.CAM_F),
               lidar_fov=(rig.LIDAR_HFOV, rig.LIDAR_EL, rig.LIDAR_RANGE), info=rig.depth.info,
               network_seconds=rig.depth.seconds, network_calls=rig.depth.sent)
    pickle.dump(out, open(path, "wb"))
    print("recorded %s: %s" % (mode, sim.message), flush=True)
    os._exit(0)


# ------------------------------------------------------------------------------------------------
# Drawing
# ------------------------------------------------------------------------------------------------

def draw(path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    data = pickle.load(open(path, "rb"))
    cam, lid = data["camera"], data["camera+lidar"]
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    C = {"front": "#1f77b4", "rear": "#d62728", "bumper": "#2ca02c", "lidar": "#9467bd"}
    NAMES = (("front", "stereo pair"), ("rear", "rear camera"), ("bumper", "bumper camera"))

    def save(fig, name):
        fig.savefig(os.path.join(IMG, name), dpi=110, bbox_inches="tight", facecolor="white")
        plt.close(fig)
        print("wrote docs/img/" + name)

    def to_ground(p, a, sg, far=7.0):                      # end of a sight line in the side view
        q = p + far * np.array([sg * math.cos(a), math.sin(a)])
        return p + (q - p) * p[1] / (p[1] - q[1]) if q[1] < 0.0 else q

    # ---- the rig -----------------------------------------------------------------------------
    fig, (ax, bx) = plt.subplots(1, 2, figsize=(15, 4.6), gridspec_kw=dict(width_ratios=[1.6, 1]))
    pr, z0 = lid["profile"], lid["ref_height"]
    # the body from the side: the envelope of the mesh along the middle of the car
    top = np.array([pr[np.abs(pr[:, 0] - x) < 0.11, 1].max() for x in pr[:, 0]]) + z0
    ax.fill_between(pr[:, 0], z0, top, color="0.82", lw=0)
    ax.axhline(0.0, color="0.3", lw=1.2)
    for c in lid["cameras"]:
        if c["label"] not in C:
            continue
        fwd = c["R"] @ np.array([1.0, 0.0, 0.0])
        pitch, sg = math.atan2(-fwd[2], math.hypot(fwd[0], fwd[1])), (1.0 if fwd[0] > 0 else -1.0)
        p = np.array([c["pos"][0], c["pos"][2] + z0])
        low = -pitch - 0.5 * lid["vfov"]
        ahead = sg * (pr[:, 0] - p[0]) > 0.3                 # the car's own body may hide the lower part
        if ahead.any():
            low = max(low, float(np.arctan2(top[ahead] - p[1], sg * (pr[ahead, 0] - p[0])).max()))
        for a in (low, -pitch + 0.5 * lid["vfov"]):
            q = to_ground(p, a, sg, 9.0)
            ax.plot([p[0], q[0]], [p[1], q[1]], color=C[c["label"]], lw=1.0)
        ax.plot(*p, "o", color=C[c["label"]], ms=6, label="%s, %.2f m up" % (dict(NAMES)[c["label"]], p[1]))
    p = np.array([lid["lidar"][0], lid["lidar"][2] + z0])
    for a in lid["lidar_fov"][1]:
        q = to_ground(p, a, 1.0)
        ax.plot([p[0], q[0]], [p[1], q[1]], color=C["lidar"], lw=1.0, ls="--")
    ax.plot(*p, "s", color=C["lidar"], ms=6, label="lidar (optional), %.2f m up" % p[1])
    ax.set_aspect("equal"); ax.set_xlim(-7.0, 9.5); ax.set_ylim(-0.2, 4.2)
    ax.set_xlabel("along the car [m]"); ax.set_ylabel("height [m]")
    ax.legend(loc="upper left", fontsize=8, frameon=False, ncol=2)
    ax.set_title("From the side: the upper and lower edge of what each sensor sees", fontsize=10)
    x0, x1, hw = lid["own"]
    bx.add_patch(plt.Rectangle((x0, -hw), x1 - x0, 2 * hw, color="0.82"))
    for c in lid["cameras"]:
        if c["label"] not in C:
            continue
        yaw = math.atan2(c["R"][1, 0], c["R"][0, 0])
        a = np.linspace(yaw - 0.5 * lid["hfov"], yaw + 0.5 * lid["hfov"], 40)
        pts = np.concatenate([[c["pos"][:2]], c["pos"][:2] + c["reach"] * np.stack([np.cos(a), np.sin(a)], 1), [c["pos"][:2]]])
        bx.fill(pts[:, 0], pts[:, 1], color=C[c["label"]], alpha=0.18, lw=0)
        bx.plot(pts[:, 0], pts[:, 1], color=C[c["label"]], lw=1.0)
    hf, _, reach = lid["lidar_fov"]
    a = np.linspace(-0.5 * hf, 0.5 * hf, 40)
    pts = np.concatenate([[lid["lidar"][:2]], lid["lidar"][:2] + reach * np.stack([np.cos(a), np.sin(a)], 1), [lid["lidar"][:2]]])
    bx.plot(pts[:, 0], pts[:, 1], color=C["lidar"], lw=1.0, ls="--")
    bx.set_aspect("equal"); bx.set_xlim(-8, 24); bx.set_ylim(-13, 13)
    bx.set_xlabel("along the car [m]"); bx.set_ylabel("to the left [m]")
    bx.set_title("From above: how far each is used for the map", fontsize=10)
    save(fig, "sensor_rig.png")

    # ---- one moment: what each camera delivers and what is made of it --------------------------
    palette = np.array([(34, 38, 48), (84, 88, 96), (255, 120, 40), (112, 44, 44), (150, 120, 70), (255, 240, 60)], np.uint8)
    t = SNAPSHOTS[1] if SNAPSHOTS[1] in cam["frames"] else sorted(cam["frames"])[-1]
    fr = cam["frames"][t]
    fig, axes = plt.subplots(3, 3, figsize=(15, 9.6))
    for row, (label, name, net) in enumerate((("front", "front left camera", cam["info"]["model"] + ", from the stereo pair"),
                                              ("rear", "rear camera", "a monocular network, anchored to the ground"),
                                              ("bumper", "bumper camera", "a monocular network, anchored to the ground"))):
        f = fr[label]
        axes[row, 0].imshow(f["image"]); axes[row, 0].set_title(name, fontsize=10)
        im = axes[row, 1].imshow(np.where(f["range"] > 0.05, f["range"], np.nan), cmap="turbo_r", vmin=0.0, vmax=15.0)
        axes[row, 1].set_title("range computed by " + net, fontsize=10)
        fig.colorbar(im, ax=axes[row, 1], fraction=0.03, pad=0.02, label="m")
        axes[row, 2].imshow(palette[f["view"]], interpolation="nearest")
        axes[row, 2].set_title("read as: ground (grey), obstacle (orange), paint (yellow), own body (red)", fontsize=9)
        for a in axes[row]:
            a.set_xticks([]); a.set_yticks([])
    fig.suptitle("%.0f s into the run" % t, fontsize=11, y=0.995)
    fig.tight_layout()
    save(fig, "camera_frame.png")

    # ---- accuracy against the scenario ---------------------------------------------------------
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.4))
    ax = axes[0]
    edges_r = np.array([0.8, 1.5, 2.5, 4.0, 6.0, 8.0, 10.0, 12.0, 15.0])
    for k, (label, name) in enumerate(NAMES):
        d = cam["road"][cam["road"][:, 0] == k]
        mids, med, p90 = [], [], []
        for lo, hi in zip(edges_r[:-1], edges_r[1:]):
            m = (d[:, 1] >= lo) & (d[:, 1] < hi)
            if m.sum() >= 200:
                e = np.abs(d[m, 2] - d[m, 1])
                mids.append(0.5 * (lo + hi)); med.append(np.median(e)); p90.append(np.quantile(e, 0.9))
        ax.plot(mids, med, "o-", color=C[label], label=name + ", median")
        ax.plot(mids, p90, "o--", color=C[label], alpha=0.6, label=name + ", 9 of 10 within")
    ax.set_yscale("log"); ax.set_xlabel("true range of the road surface [m]"); ax.set_ylabel("range error [m]")
    ax.set_title("Range of the road, against the plane that it is", fontsize=10); ax.legend(fontsize=8, frameon=False)
    ax = axes[1]
    for src, k, label, name in ((cam, 0, "front", "stereo pair"), (cam, 1, "rear", "rear camera"),
                                (cam, 2, "bumper", "bumper camera"), (lid, 3, "lidar", "lidar")):
        d = src["hits"][src["hits"][:, 0] == k]
        d = d[d[:, 1] < 30.0]
        if len(d) > 100:
            e = d[:, 2] - d[:, 1]
            ax.hist(np.clip(e, -0.5, 0.5), bins=np.linspace(-0.5, 0.5, 81), histtype="step", density=True, color=C[label],
                    label="%s: median %+.0f cm, half within %.0f cm" % (name, 100 * np.median(e), 100 * np.median(np.abs(e))))
    ax.set_xlabel("nearest obstacle per bearing: reported minus true range [m]"); ax.set_yticks([])
    ax.set_title("Obstacles, against the outlines of the cars and kerbs", fontsize=10); ax.legend(fontsize=8, frameon=False)
    ax = axes[2]
    for k, (label, name) in enumerate(NAMES):
        d = cam["segs"][cam["segs"][:, 0] == k] if len(cam["segs"]) else np.zeros((0, 3))
        if len(d):
            ax.hist(np.clip(d[:, 2], 0, 0.5), bins=np.linspace(0, 0.5, 51), weights=d[:, 1], histtype="step", color=C[label],
                    label="%s: %.0f%% of %.0f m on a painted line" % (name, 100 * d[d[:, 2] < 0.12, 1].sum() / d[:, 1].sum(), d[:, 1].sum()))
    ax.set_xlabel("largest distance of a reported segment from a painted line [m]"); ax.set_ylabel("metres of segments")
    ax.set_title("Line segments, against the painted lines", fontsize=10); ax.legend(fontsize=8, frameon=False)
    fig.tight_layout()
    save(fig, "sensor_accuracy.png")

    # ---- the maps ----------------------------------------------------------------------------
    fig, axes = plt.subplots(1, 2, figsize=(16, 6.2))
    for ax, d, name in zip(axes, (cam, lid), ("cameras", "cameras and lidar")):
        x0, y0, res, nx, ny = d["grid"]
        T = lambda a: a.T if a.shape == (nx, ny) else a
        img = np.full((ny, nx, 3), 0.13)
        img[T(d["far"] >= 3)] = (0.25, 0.3, 0.36)
        img[T(d["free"] >= 1)] = (0.42, 0.45, 0.5)
        img[T(d["obstacles"] >= 2)] = (0.95, 0.35, 0.25)
        ax.imshow(img, origin="lower", extent=(x0, x0 + nx * res, y0, y0 + ny * res), interpolation="nearest")
        for poly in d["cars"]:
            ax.plot(*np.vstack([poly, poly[:1]]).T, color="0.75", lw=0.7)
        for ends in d["tracks"]:
            ax.plot(*np.array(ends).T, color="cyan", lw=1.8)
        ax.plot(d["trail"][:, 0], d["trail"][:, 1], color="white", lw=1.0, ls=":")
        ax.plot(*np.vstack([d["parked"], d["parked"][:1]]).T, color="white", lw=1.4)
        xs = [v for ln in d["lines"] for v in (ln[0], ln[2])]; ys = [v for ln in d["lines"] for v in (ln[1], ln[3])]
        ax.set_xlim(min(xs) - 10, max(xs) + 6); ax.set_ylim(min(ys) - 3, max(ys) + 3); ax.set_aspect("equal")
        ax.set_title("%s: the map when parked\nred: obstacle, grey: seen free, dark blue: probably free, cyan: lines" % name, fontsize=9)
    fig.tight_layout()
    save(fig, "sensor_maps.png")

    # ---- numbers for the text ------------------------------------------------------------------
    for d in (cam, lid):
        print("%s: %s; networks %.0f s in %d calls (%s, %s)" % (d["mode"], d["message"], d["network_seconds"], d["network_calls"],
                                                             d["info"]["model"], d["info"]["mono"]))
    for k, (label, name) in enumerate(NAMES):
        d = cam["road"][cam["road"][:, 0] == k]
        for lo, hi in ((0.8, 2), (2, 4), (4, 7), (7, 10), (10, 15)):
            m = (d[:, 1] >= lo) & (d[:, 1] < hi)
            if m.sum() >= 200:
                e = np.abs(d[m, 2] - d[m, 1])
                print("  road, %-13s %4.1f-%4.1f m: median %.3f m, 9 of 10 within %.3f m (%d samples)" % (
                    name, lo, hi, np.median(e), np.quantile(e, 0.9), m.sum()))
    for src, k, name in ((cam, 0, "stereo pair"), (cam, 1, "rear camera"), (cam, 2, "bumper camera"), (lid, 3, "lidar")):
        d = src["hits"][src["hits"][:, 0] == k]
        d = d[d[:, 1] < 30.0]
        if len(d) > 100:
            e = d[:, 2] - d[:, 1]
            print("  obstacles, %-13s median %+.3f m, half within %.3f m, 9 of 10 within %.3f m (%d bearings)" % (
                name, np.median(e), np.median(np.abs(e)), np.quantile(np.abs(e), 0.9), len(d)))
    for k, (label, name) in enumerate(NAMES):
        d = cam["segs"][cam["segs"][:, 0] == k] if len(cam["segs"]) else np.zeros((0, 3))
        if len(d):
            print("  lines, %-13s %d segments, %.0f m, %.1f%% of the length on a painted line, median offset %.3f m" % (
                name, len(d), d[:, 1].sum(), 100 * d[d[:, 2] < 0.12, 1].sum() / d[:, 1].sum(), np.median(d[:, 2])))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "record-one":
        record_one(sys.argv[2], sys.argv[3])
    elif len(sys.argv) > 1 and sys.argv[1] == "record":
        record(sys.argv[2])
    elif len(sys.argv) > 1 and sys.argv[1] == "draw":
        draw(sys.argv[2])
    else:
        import tempfile
        tmp = os.path.join(tempfile.gettempdir(), "parking_sensor_frames.pkl")
        record(tmp)
        draw(tmp)
