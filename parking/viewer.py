"""The window: live views of the scene with what the car knows drawn in."""

import collections
import math
import os
import time

import numpy as np
import pychrono.irrlicht as irr

from .chrono_env import chrono
from .config import A_BRAKE, A_DRIVE_MAX, PERCEPTION_DT, STEP
from .draw import _BITS, _BLANK, _RUNS
from .geometry import ego_poly, footprint_hits, rect_poly, wrap
from .inputs import MouseKeys
from .stalls import Slot
from .vehicle import EGO
from .viewer_panel import PanelMixin
from .viewer_pictures import PicturesMixin


class Viewer(PicturesMixin, PanelMixin):
    COLORS = dict(det=(1.0, 0.9, 0.1), track=(0.1, 0.9, 1.0), free=(0.2, 1.0, 0.3), occupied=(1.0, 0.25, 0.2),
                  unknown=(0.6, 0.6, 0.6), fwd=(0.3, 0.55, 1.0), rev=(1.0, 0.35, 0.9), goal=(1.0, 1.0, 1.0),
                  scan=(1.0, 0.45, 0.1), box=(1.0, 1.0, 1.0), box_bad=(1.0, 0.2, 0.2), mpc=(1.0, 0.95, 0.2),
                  fan=(0.45, 0.6, 0.95), stub=(0.1, 0.55, 0.65))
    CAM_PALETTE = {0: (34, 38, 48), 1: (84, 88, 96), 2: (255, 120, 40), 3: (112, 44, 44), 4: (150, 120, 70),
                   5: (255, 240, 60)}
    DEPTH_SCALE = 15.0         # range at the far end of the colour scale of the depth pictures [m]
    HEIGHT_SCALE = 1.6         # height at the top of the colour scale of the range picture [m]
    RANGE_SPAN = 13.0          # what the range picture shows to each side of the car [m]

    def __init__(self, sim, args):
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
        self.pics, self.tex, self.pic_stamp, self.range_bg = [], {}, None, None
        self.canvas = None             # (image, x, y of its corner) while drawing into a picture, not the window
        self.panel_key, self.panel_wall = None, 0.0
        self.add_texture = self._texture_call()
        try:
            if self.add_texture is not None:
                self.drv.removeTexture(self._upload("probe", np.zeros((2, 2, 3), np.uint8)))
        except Exception:
            self.add_texture = None
        if args.layout == "sensors" and self.add_texture is None:
            print("[parking] Python cannot put pictures into the window with this Irrlicht library: the views "
                  "show the scene, not the sensor images", flush=True)
        if args.layout == "sensors" and self.add_texture is not None:
            # The viewer's own top and chase views on the left. On the right what the cameras
            # deliver and what the stereo network makes of the front pair. Below the top view, every
            # range that was computed or measured, from above.
            rig = sim.sensor
            ch = (H - top) // 4
            xl = W - ch * rig.CAM_W // rig.CAM_H
            ym = top + int(0.52 * (H - top))
            self.rects = [(0, top, xl, ym), (0, ym, xl // 2, H)]
            size = "ZED X ONE GS, %d X %d" % (rig.CAM_W, rig.CAM_H)
            net = rig.depth.info["model"].upper()
            for i, (key, title, sub) in enumerate((
                    (("front", "image"), "FRONT LEFT CAMERA", size + ", ONE OF THE STEREO PAIR"),
                    (("front", "range"), "STEREO DEPTH: " + net, "COMPUTED FROM THE TWO FRONT IMAGES"),
                    (("rear", "image"), "REAR CAMERA", size), (("bumper", "image"), "BUMPER CAMERA", size))):
                self.pics.append((key, (xl, top + i * ch, W, top + (i + 1) * ch), title, sub))
            what = "FROM STEREO, FAINT: FROM ONE CAMERA"
            if rig.lidar is not None:
                what = "LARGE DOTS: LIDAR %d X %d, SMALL: STEREO" % (rig.LIDAR_W, rig.LIDAR_H)
            self.pics.append((("range", ""), (xl // 2, ym, xl, H), "RANGES FROM ABOVE", what))
            self.pic_rects = [(irr.recti(*r), irr.recti(0, 0, r[2] - r[0], r[3] - r[1])) for _, r, _, _ in self.pics]
            rig.show = True
        elif args.layout == "wide":    # big top view on the left, three views stacked on the right
            xs, hh = int(0.64 * W), (H - top) // 3
            self.rects = [(0, top, xs, H), (xs, top, W, top + hh), (xs, top + hh, W, top + 2 * hh),
                          (xs, top + 2 * hh, W, H)]
        else:
            xm, ym = W // 2, top + (H - top) // 2
            self.rects = [(0, top, xm, ym), (xm, top, W, ym), (0, ym, xm, H), (xm, ym, W, H)]
        self.labels = ["TOP VIEW", "CHASE VIEW", "FRONT VIEW", "STALL VIEW"]      # the viewer's own cameras

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
        self.cam_stamp, self.cam_rects = None, []       # cached picture of what the cameras are read as
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
        if self.canvas is not None:        # into a picture that is uploaded later, see _panel and _pictures
            img, ox, oy = self.canvas
            part = img[max(int(y0) - oy, 0):max(int(y1) - oy, 0), max(int(x0) - ox, 0):max(int(x1) - ox, 0)]
            c = color.color                # alpha, red, green, blue in one number
            rgb = ((c >> 16) & 255, (c >> 8) & 255, c & 255)
            if c >> 24 == 255:
                part[:] = rgb
            else:
                part[:] = (part.astype(np.uint16) * (255 - (c >> 24)) + np.array(rgb, np.uint16) * (c >> 24)) // 255
            return
        key = (x0, y0, x1, y1)
        r = self.rect_cache.get(key)
        if r is None:
            if len(self.rect_cache) > 20000:
                self.rect_cache.clear()
            r = self.rect_cache[key] = irr.recti(int(x0), int(y0), int(x1), int(y1))
        self.drv.draw2DRectangle(color, r)

    def _text(self, text, x, y, scale=2, rgb=(255, 255, 255), alpha=255):
        if self.canvas is not None and text:           # into a picture: all glyphs at once
            img, ox, oy = self.canvas
            mask = np.hstack([_BITS.get(ch, _BLANK) for ch in text.upper()])
            mask = mask.repeat(scale, axis=0).repeat(scale, axis=1)
            x, y = int(x) - ox, int(y) - oy
            if x >= 0 and y >= 0:
                part = img[y:y + mask.shape[0], x:x + mask.shape[1]]
                part[mask[:part.shape[0], :part.shape[1]]] = rgb
            return
        col = irr.SColor(alpha, *rgb)
        for ch in text.upper():
            for row, c0, c1 in _RUNS.get(ch, ()):
                self._rect(col, x + c0 * scale, y + row * scale, x + c1 * scale, y + (row + 1) * scale)
            x += 6 * scale

    # ---- scene overlays --------------------------------------------------------

    def _rebuild_items(self):
        sim, C = self.sim, self.COLORS
        self.items = []
        for x1, y1, x2, y2, *_ in sim.dets:
            self._line([(x1, y1), (x2, y2)], C["det"], z=0.07, thick=1)
        for origin, head, half, reach in getattr(sim.sensor, "fans", ()):      # what each sensor looks at
            arc = [(origin[0] + reach * math.cos(head + a), origin[1] + reach * math.sin(head + a))
                   for a in np.linspace(-half, half, max(3, int(half / 0.12)))]
            pts = np.array(arc if half > 3.0 else [tuple(origin)] + arc + [tuple(origin)])
            self.scan_items.append((self._vec(pts, 0.3), chrono.ChColor(*C["fan"])))      # top view only
        confirmed = sim.lines.confirmed()
        for t in sim.lines.markers():
            self._line(t.ends(), C["track"] if t in confirmed else C["stub"], z=0.04)
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
        self.labels[2] = "REAR VIEW" if reverse else "FRONT VIEW"
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
            self.labels[3] = "STALL VIEW"
            cam.setPosition(V(float(eye[0]), float(eye[1]), 5.5))
            cam.setTarget(V(float(aim[0]), float(aim[1]), 0.3))
        else:
            self.labels[3] = "SIDE VIEW"
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
        self._pictures()
        self._hud()
        self.vis.EndScene()

    def _hud(self):
        sim, W, H = self.sim, self.W, self.H
        dark, edge = irr.SColor(255, 18, 20, 24), irr.SColor(255, 8, 8, 10)
        self._rect(dark, 0, 0, W, 30)
        for x0, y0, x1, y1 in self.rects + [r for _, r, _, _ in self.pics]:      # frames around the views
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

        # the three commands sent to the car, in physical units
        steer, drive, brake = sim.cmd
        x0, y0 = 14, H - 54
        self._rect(irr.SColor(150, 0, 0, 0), x0 - 6, y0 - 6, x0 + 330, y0 + 46)
        rows = (("STEER", steer / EGO.steer_max, (80, 170, 255), "%+5.1f DEG" % math.degrees(steer)),
                ("DRIVE", -drive / EGO.torque(A_DRIVE_MAX), (90, 230, 110) if drive >= 0 else (255, 110, 235),
                 "%+5.0f NM" % drive),
                ("BRAKE", -brake / EGO.torque(A_BRAKE), (240, 80, 70), "%5.0f NM" % brake))
        for k, (name, frac, col, text) in enumerate(rows):        # bars grow from the middle, left = positive
            y, mid = y0 + 14 * k, x0 + 145
            self._text(name, x0, y, 1)
            self._rect(irr.SColor(255, 70, 70, 70), x0 + 50, y, x0 + 240, y + 8)
            end = mid - 95 * min(max(frac, -1.0), 1.0)
            self._rect(irr.SColor(255, *col), min(mid, end), y, max(mid, end) + 1, y + 8)
            self._text(text, x0 + 248, y, 1)

        r = self.rects[0]
        legend = (("LINE DETECTIONS", "det"), ("LINE MAP", "track"), ("FREE STALL", "free"), ("OCCUPIED", "occupied"),
                  ("PATH FWD", "fwd"), ("PATH REV", "rev"), ("RANGE SCAN", "scan"))
        x, y = r[0] + (350 if r[3] > H - 60 else 270), r[3] - 18       # clear of the command read-out
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
