"""The sensor rig: Chrono::Sensor cameras and lidar on the car, the networks that compute depth
from the images, and what is made of both."""

import collections
import math
import os

import numpy as np

from .chrono_env import chrono, sens
from .config import PERCEPTION_DT
from .perception import _rot_y, _rot_z, angular_rays, paint_segments, pinhole_rays, planar_scan
from .scene_net import KERB, MARKING, OWN
from .vehicle import EGO
from .world import light_scene


def _grown(mask, k):
    """A mask and everything within k cells of it, along rows and columns."""
    out = mask.copy()
    for axis in (0, 1):
        src = out.copy()
        for step in range(1, k + 1):
            a, b = [slice(None)] * 2, [slice(None)] * 2
            a[axis], b[axis] = slice(step, None), slice(None, -step)
            out[tuple(a)] |= src[tuple(b)]
            out[tuple(b)] |= src[tuple(a)]
    return out


def sensor_mounts(body):
    """Where the sensors go, read from the car's body mesh, as (x, z) in the chassis frame:
    'stereo' 4.5 cm inside the windshield and 20 cm below its top edge, 'rear' at the top of the
    tail, 'bumper' on the nose, 'lidar' on the roof above the windshield."""
    shape = chrono.CastToChVisualShapeTriangleMesh(body.GetVisualModel().GetShape(0))
    path = shape.GetMesh().GetFileName()
    glass, mat = set(), None
    with open(os.path.splitext(path)[0] + ".mtl") as f:    # glass is what the materials call see-through
        for line in f:
            tok = line.split()
            if tok[:1] == ["newmtl"]:
                mat = tok[1]
            elif tok[:1] == ["d"] and float(tok[1]) < 0.3:
                glass.add(mat)
    verts, panes, mat = [], [], None
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                verts.append([float(v) for v in line.split()[1:4]])
            elif line.startswith("usemtl"):
                mat = line.split()[1]
            elif line.startswith("f ") and mat in glass:
                panes += [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
    verts = np.array(verts)
    mid = verts[np.abs(verts[:, 1]) < 0.3]                  # the strip along the middle of the car
    x_rear, x_front = verts[:, 0].min(), verts[:, 0].max()
    g = verts[np.unique(panes)]
    g = g[(np.abs(g[:, 1]) < 0.25) & (g[:, 0] > 0.5 * (g[:, 0].min() + g[:, 0].max()))]     # windshield, middle strip
    slope = np.polyfit(g[:, 0], g[:, 2], 1)[0]
    along = np.array([1.0, slope]) / math.hypot(1.0, slope)
    top = g[g[:, 2].argmax(), [0, 2]]
    stereo = top + 0.20 * along - 0.045 * np.array([-along[1], along[0]])
    tail = mid[mid[:, 0] < x_rear + 0.15]
    nose = mid[mid[:, 0] > x_front - 0.12]
    roof = mid[np.abs(mid[:, 0] - (top[0] - 0.05)) < 0.08]
    z_tail = tail[:, 2].max() - 0.03
    return dict(stereo=(float(stereo[0]), float(stereo[1])),
                rear=(float(tail[tail[:, 2] > z_tail - 0.12, 0].min() - 0.02), float(z_tail)),
                bumper=(float(x_front + 0.02), float(np.quantile(nose[:, 2], 0.75))),
                lidar=(float(top[0] - 0.05), float(roof[:, 2].max() + 0.07)))


class SensorRig:
    """The car's sensors, simulated with Chrono::Sensor, and what is computed from their data.

    camera        Four cameras, each a Stereolabs ZED X One GS with the 2.2 mm lens in its
                  960 x 600 mode: two as a stereo pair behind the top of the windshield, one at
                  the top of the tail looking back and down, one on the front bumper. Range comes
                  from the images. A stereo network (IGEV++) matches the pair, and a monocular
                  depth network (Depth Anything V2), anchored to the ground, covers what the two
                  single cameras see. Painted lines come from each image, seen from above.
    camera+lidar  the same plus a forward-facing lidar on the roof above the windshield
    """
    MODES = ("camera", "camera+lidar")

    CAM_W, CAM_H = 960, 600                  # the sensor's 1920 x 1200 pixels, binned 2 x 2
    CAM_F = 2.2e-3 / 6.0e-6                  # focal length [pixels]: 2.2 mm lens, 3 micron pixels, binned
    CAM_HFOV = 2.0 * math.atan(0.5 * CAM_W / CAM_F)          # 105 degrees for the rectified image
    BASELINE = 0.30                          # between the two cameras of the stereo pair [m]
    REAR_PITCH, BUMPER_PITCH = math.radians(25.0), math.radians(5.0)      # downwards
    STEREO_LAG, MONO_LAG = 0.2, 0.1          # how long a network's answer takes to arrive [s]
    DISP_ERR = 0.25                          # disparity error that the processing assumes [pixels]
    MONO_ERR = (0.02, 0.07)                  # range error assumed for monocular depth: 2 cm + 7 %
    MONO_RANGE = 3.0                         # ground seen by a single camera counts as probably free up to here
    CAM_FAR = 30.0             # what is further than this, or the sky, is reported at this range
    CAM_RANGE = 12.0           # obstacles and free ground are taken from a depth image up to here at most
    PAINT_RANGE = 11.0         # painted lines are looked for up to here in the image of the stereo pair,
    MONO_PAINT = {"rear": 5.0, "bumper": 3.5}       # and up to here in a single camera's: it is lower, and
    #                                                 # only the ground tells how far away a pixel is
    RANGE_TOL, HEIGHT_TOL = 0.15, 0.04     # errors beyond which a depth image does not settle what a cell is
    LIDAR_W, LIDAR_H, LIDAR_HFOV = 480, 32, math.radians(120.0)
    LIDAR_EL = (math.radians(-20.0), math.radians(5.0))
    LIDAR_RANGE = 20.0
    Z_GROUND, Z_OBSTACLE, Z_TOP = 0.05, 0.08, 2.3     # a kerb is 0.15 m high
    Z_LIDAR = 0.30             # lidar returns below this are not placed in the map as obstacles
    # With the scene network (scene_net.py):
    SCENE_TOP = 224            # rows of the stereo image above this one, from the top, are sky: not sent to it
    PAINT_HINT = 1.2           # where it sees a marking, paint has to be this much lighter than the ground (not 1.8)
    KERB_RANGE = {"stereo": 13.0, "mono": 4.0}      # a kerb it sees is placed up to here [m]
    SCENE_MONO = 2             # it gets every second frame that the monocular network gets: what the single
    #                            cameras add (a kerb behind the car, faint paint beside it) is used at walking pace

    def __init__(self, world, mode, noise, rng, depth, sky, stereo_hz=5.0, mono_hz=5.0, stereo_rows=None, given=()):
        self.mode, self.noise, self.rng, self.depth = mode, noise, rng, depth
        # (the worker says whether it has the network that labels markings, kerbs and the car's own body)
        self.scene = depth is not None and bool(depth.info.get("scene"))
        self.kerbs = []                                   # the kerbs in each camera frame of the last call of sense: (seconds, points)
        # How the car leans and how high it rides come from the road the stereo pair sees
        # (_road_plane), unless it is told ('attitude' in given).
        self.own_attitude = "attitude" not in given
        self.lean, self.height = np.eye(3), None          # chassis frame to a frame level with the road, and its height above it
        self.plane = None                                 # the road in the chassis frame: z = a x + b y + c
        self.believed = collections.OrderedDict()         # where the car thought it was at the time of each render
        # The rows of the stereo images that go to the network, counted from the top. The top of
        # an image is sky and the bottom is the car's own bonnet, and the network spends as long
        # on those rows as on the road. (Even numbers: the range image is made of 2 x 2 blocks.)
        top, bottom = stereo_rows or (0, self.CAM_H)
        self.rows = (max(0, top - top % 2), min(self.CAM_H, bottom + bottom % 2))
        # How often each network runs is a matter of how much computing there is, not of the
        # method: the maps add up the time each answer stands for, not the number of answers.
        # The cameras can deliver a frame per perception tick.
        self.stereo_every = max(1, int(round(1.0 / (stereo_hz * PERCEPTION_DT))))
        self.mono_every = max(1, int(round(1.0 / (mono_hz * PERCEPTION_DT))))
        self.name = "Chrono::Sensor " + mode.replace("+", " + ")
        self.body = world.car.GetChassisBody()
        self.ground = world.ground
        self.system = world.system
        # the car's outline in the chassis frame: its own body shows up in every sensor
        mid = -EGO.ref_to_rear
        self.own = (mid - EGO.rear, mid + EGO.front, EGO.half_width)
        mounts = sensor_mounts(self.body)
        k = noise

        # (the renderer's own random numbers, with which OptiX spreads the rays of a pixel: the
        # same in every run of a scenario, so that a run can be repeated)
        sens.ChSensorManager.SetRandomSeed(1 + getattr(world.scn, "seed", 0))
        self.manager = sens.ChSensorManager(self.system)
        exposure, vignette = light_scene(self.manager.scene, sky)
        rate = 1.05 / PERCEPTION_DT        # a little faster than it is read, so every read is fresh
        W, H = self.CAM_W, self.CAM_H
        # Exposure and the darker corners of the lens, where the renderer does not do them: both
        # scale the light, and a pixel value is the light to the power 1 / 2.2
        self.lens = None
        if exposure != 1.0 or vignette != 0.0:
            px, py = np.meshgrid((np.arange(W) + 0.5) / W * 2.0 - 1.0, (np.arange(H) + 0.5) / H * 2.0 - 1.0)
            light = exposure * np.maximum(1.0 - vignette * (px * px + py * py), 0.0)
            self.lens = (light ** (1.0 / 2.2)).astype(np.float32)[..., None]
        self.rays = pinhole_rays(W // 2, H // 2, self.CAM_HFOV)        # of the half-size depth images
        self.cameras, self.lidar = [], None

        def camera(label, x, y, z, yaw, pitch, role, skew=(0.0, 0.0)):
            # 'skew' is what the mounting is off by and the processing does not know about
            q = chrono.QuatFromAngleZ(yaw + skew[0]) * chrono.QuatFromAngleY(pitch + skew[1])
            s = sens.ChCameraSensor(self.body, rate, chrono.ChFramed(chrono.ChVector3d(x, y, z), q), W, H,
                                    self.CAM_HFOV, 2)
            s.PushFilter(sens.ChFilterRGBA8Access())
            self._add(s, label + " camera")
            cam = dict(label=label, role=role, sensor=s, pos=np.array([x, y, z]), R=_rot_z(yaw) @ _rot_y(pitch),
                       stamp=-1.0, gain=1.0 + 0.03 * k * float(rng.normal()), half=0.5 * self.CAM_HFOV)
            self.cameras.append(cam)
            return cam

        x, z = mounts["stereo"]
        self.left = camera("front", x, 0.5 * self.BASELINE, z, 0.0, 0.0, "stereo")
        # the pair is never mounted exactly parallel: what is left after calibration, about 0.015 degrees
        self.right = camera("right", x, -0.5 * self.BASELINE, z, 0.0, 0.0, "partner",
                            tuple(math.radians(0.015) * k * rng.normal(size=2)))
        camera("rear", mounts["rear"][0], 0.0, mounts["rear"][1], math.pi, self.REAR_PITCH, "mono")
        camera("bumper", mounts["bumper"][0], 0.0, mounts["bumper"][1], 0.0, self.BUMPER_PITCH, "mono")
        self.left["reach"], self.left["err"] = self.CAM_RANGE, (0.0, 0.0, self.DISP_ERR / (self.CAM_F * self.BASELINE))
        self.left["paint"] = self.PAINT_RANGE
        for cam in self.cameras[2:]:
            cam["reach"], cam["err"], cam["half"] = self.MONO_RANGE, self.MONO_ERR + (0.0,), math.radians(80.0)
            cam["paint"] = self.MONO_PAINT[cam["label"]]
        if mode == "camera+lidar":
            pos = np.array([mounts["lidar"][0], 0.0, mounts["lidar"][1]])
            self.lidar = dict(pos=pos, stamp=-1.0, rays=angular_rays(self.LIDAR_W, self.LIDAR_H, self.LIDAR_HFOV,
                                                                     *self.LIDAR_EL, inclusive=True))
            self.lidar["sensor"] = sens.ChLidarSensor(
                self.body, rate, chrono.ChFramed(chrono.ChVector3d(*pos), chrono.QUNIT), self.LIDAR_W, self.LIDAR_H,
                self.LIDAR_HFOV, self.LIDAR_EL[1], self.LIDAR_EL[0], 1.5 * self.LIDAR_RANGE)
            self.lidar["sensor"].PushFilter(sens.ChFilterDIAccess())
            self._add(self.lidar["sensor"], "lidar")

        # Sensor noise per count of the 8 bit image. In linear light it is shot noise plus read
        # noise, here with a signal to noise ratio of 40 at mid grey for noise scale 1. The gamma
        # curve stretches it in the shadows.
        v = np.maximum(np.arange(256), 4) / 255.0
        lin = v ** 2.2
        self.sigma = (k * 255.0 * np.sqrt(1.03e-4 * lin + 1.44e-6) / (2.2 * v ** 1.2)).astype(np.float32)
        self.grain = rng.standard_normal((3, H, W, 3), dtype=np.float32) if k > 0.0 else None
        self.linear = ((np.arange(256) / 255.0) ** 2.2).astype(np.float32)     # brightness of a count

        self.tick, self.pending = 0, []
        self.frames = collections.OrderedDict()      # chassis frame at the time of each render
        self.paint = np.zeros((0, 2))                 # for the viewer: where paint was seen last
        self.fans = []                                # for the viewer: (origin, heading, half fov, range) per sensor
        self.show = False                             # set by the viewer: keep what each sensor delivered

    def _add(self, sensor, name):
        sensor.SetName(name)
        sensor.SetLag(0.0)
        sensor.SetCollectionWindow(0.0)
        self.manager.AddSensor(sensor)

    def _frame(self):
        f = self.body.GetFrameRefToAbs()
        p, R = f.GetPos(), f.GetRotMat()
        ax, ay, az = R.GetAxisX(), R.GetAxisY(), R.GetAxisZ()
        return np.array([p.x, p.y, p.z]), np.array([[ax.x, ay.x, az.x], [ax.y, ay.y, az.y], [ax.z, ay.z, az.z]])

    def _believed_frame(self, pose, tilt):
        """The chassis frame as the car believes it to be: position and heading from its own
        estimate, pitch and roll off by `tilt`. The car takes the road to be a plane and itself
        to stand on it: it knows its height above the road under it and how it leans relative
        to that road, not how the road itself lies. On an uneven road the ground further away
        is then not where the car expects it."""
        p, R = self._frame()
        sx, sy = self.ground.slope(p[0], p[1])
        n = np.array([-sx, -sy, 1.0]) / math.sqrt(sx * sx + sy * sy + 1.0)       # the road's normal under the car
        k = np.array([-n[1], n[0], 0.0])                                         # tips the vertical onto it
        K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
        lie = np.eye(3) + K + K @ K / (1.0 + n[2])
        R = lie.T @ R                                                            # how the car leans relative to the road
        turn = pose[2] - math.atan2(R[1, 0], R[0, 0])
        c, s = math.cos(tilt[1]), math.sin(tilt[1])
        roll = np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
        return (np.array([pose[0] + EGO.ref_to_rear * math.cos(pose[2]), pose[1] + EGO.ref_to_rear * math.sin(pose[2]),
                          p[2] - self.ground.height(p[0], p[1])]),
                _rot_z(turn) @ R @ _rot_y(tilt[0]) @ roll)

    def _own_frame(self, pose):
        """The chassis frame from what the car itself knows: position and heading from its pose
        estimate, lean and height from the road as the stereo pair last saw it."""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return (np.array([pose[0] + EGO.ref_to_rear * c, pose[1] + EGO.ref_to_rear * s, self.height or 0.0]),
                _rot_z(pose[2]) @ self.lean)

    def _road_plane(self, cam, depth):
        """How the car leans and how high it rides, from the road in a depth image: a plane is
        fitted to the points of the road 1 to 10 m ahead of the bumper, in the chassis frame.
        Nothing else tells the car its pitch, its roll or its height: not under braking, when
        the nose dips by a degree, and not on an uneven road, where the plane is that of the
        road ahead. If too little road is in view, the last plane stays."""
        P = cam["pos"].astype(np.float32) + depth[..., None] * (self.rays @ cam["R"].T.astype(np.float32))
        x0, x1, hw = self.own
        ok = (depth > 0.3) & (P[..., 0] > x1 + 1.0) & (P[..., 0] < x1 + 10.0) & (np.abs(P[..., 1]) < 5.0)
        pts = P[ok]
        if len(pts) < 2000:
            return
        pts = pts[::max(1, len(pts) // 8000)].astype(np.float64)
        A = np.column_stack([pts[:, 0], pts[:, 1], np.ones(len(pts))])
        if self.plane is None:
            # the first time: the road is the lowest level surface in view
            plane, gates = np.array([0.0, 0.0, np.quantile(pts[:, 2], 0.15)]), (0.15, 0.06, 0.03)
        else:
            plane, gates = self.plane, (0.08, 0.04, 0.025)
        for gate in gates:
            m = np.abs(pts[:, 2] - A @ plane) < gate
            if m.sum() < 800 or np.ptp(pts[m, 0]) < 2.0:
                return
            plane = np.linalg.lstsq(A[m], pts[m, 2], rcond=None)[0]
        a, b, c = plane
        if abs(a) > 0.08 or abs(b) > 0.08:
            return                             # no road lies at 5 degrees to the car
        n = np.array([-a, -b, 1.0]) / math.sqrt(a * a + b * b + 1.0)       # the road's normal in the chassis frame
        K = np.array([[0.0, 0.0, -n[0]], [0.0, 0.0, -n[1]], [n[0], n[1], 0.0]])       # (turns it onto the vertical)
        self.plane, self.lean, self.height = plane, np.eye(3) + K + K @ K / (1.0 + n[2]), -c * n[2]

    def _develop(self, cam, rgba):
        """What a camera delivers for a rendered frame: its own exposure and the noise of its
        sensor, different in every camera and every frame."""
        img = rgba[..., :3]
        if self.lens is not None:
            img = np.minimum(img * self.lens + 0.5, 255.0).astype(np.uint8)
        if self.grain is None:
            return np.array(img)
        n = self.grain[int(self.rng.integers(len(self.grain)))]
        n = np.roll(n, (int(self.rng.integers(self.CAM_H)), int(self.rng.integers(self.CAM_W))), axis=(0, 1))
        return np.clip(img * np.float32(cam["gain"]) + n * self.sigma[img] + 0.5, 0.0, 255.0).astype(np.uint8)

    def sense(self, pose, tilt=None):
        """Render the sensors for the current state of the simulation and process what they give.
        Returns (scans, line detections). pose is where the car thinks its rear axle is, and tilt
        the error of the pitch and roll it assumes (None: it knows all of that exactly). What the
        sensors show is put into the map with those, not with the truth."""
        t = round(self.system.GetChTime(), 4)
        if self.own_attitude:
            self.believed[t] = pose
            ref_p, ref_R = self.frames[t] = self._own_frame(pose)
        else:
            exact = tilt is None and self.ground.amp == 0.0
            ref_p, ref_R = self.frames[t] = self._frame() if exact else self._believed_frame(pose, tilt or (0.0, 0.0))
        while len(self.frames) > 12:
            self.frames.popitem(last=False)
        while len(self.believed) > 12:
            self.believed.popitem(last=False)
        self.tick += 1
        # Rendering is most of the work of a run. The cameras are therefore only rendered on the
        # ticks at which a network takes their frame. A lidar is read on every tick.
        take_stereo, take_mono = self.tick % self.stereo_every == 0, self.tick % self.mono_every == 0
        if take_stereo or take_mono or self.lidar is not None:
            self.manager.Update()
        fresh = set()
        for cam in self.cameras:
            buf = cam["sensor"].GetMostRecentRGBA8Buffer()
            if not buf.HasData():
                continue
            stamp = round(buf.TimeStamp, 4)
            if stamp != cam["stamp"] and stamp in self.frames:
                cam["stamp"], cam["image"] = stamp, self._develop(cam, buf.GetRGBA8Data())
                fresh.add(cam["label"])
        self.fans = [((ref_p + ref_R @ c["pos"])[:2], math.atan2(*(ref_R @ c["R"])[1::-1, 0]), min(c["half"], 1.2),
                      c["reach"]) for c in self.cameras if "reach" in c]

        # The networks do not run on every frame, and their answer takes time. A request is made
        # here and its result is used a fixed number of ticks later, with the pose the car had
        # when the images were taken.
        monos = self.cameras[2:]
        # (the quicker answer is asked for first: the answers come back in the order of the requests)
        if take_mono and all(c["label"] in fresh for c in monos):
            ident = self.depth.submit("mono", [c["image"][::-1] for c in monos])
            also = None
            if self.scene and (self.tick // self.mono_every) % self.SCENE_MONO == 0:
                also = self.depth.submit("scene", [c["image"][::-1] for c in monos])
            self.pending.append((self.tick + int(round(self.MONO_LAG / PERCEPTION_DT)), ident, monos, monos[0]["stamp"],
                                 [c["image"] for c in monos], also))
        if take_stereo and {"front", "right"} <= fresh and \
                self.left["stamp"] == self.right["stamp"]:
            r0, r1 = self.rows
            ident = self.depth.submit("stereo", [self.left["image"][::-1][r0:r1], self.right["image"][::-1][r0:r1]])
            also = self.depth.submit("scene", [self.left["image"][::-1][self.SCENE_TOP:]]) if self.scene else None
            self.pending.append((self.tick + int(round(self.STEREO_LAG / PERCEPTION_DT)), ident, [self.left], self.left["stamp"],
                                 [self.left["image"]], also))
        scans, dets, paint, self.kerbs = [], [], [], []
        while self.pending and self.pending[0][0] <= self.tick:
            _, ident, cams, stamp, images, also = self.pending.pop(0)
            maps = self.depth.collect(ident)
            labels = [None] * len(cams)
            if also is not None:           # (as the images are: the whole frame, bottom row first)
                labels = [np.concatenate([np.zeros((self.CAM_H - len(m), self.CAM_W), np.uint8), m])[::-1]
                          for m in self.depth.collect(also)]
            for cam, image, out, seen in zip(cams, images, maps, labels):
                frame = self.frames[stamp]
                if cam["role"] == "stereo":
                    rng_img = self._stereo_rows(out)
                    if self.own_attitude:      # this answer shows the road: take the car's lean and height from it
                        self._road_plane(cam, rng_img)
                        frame = self.frames[stamp] = self._own_frame(self.believed[stamp])
                elif self.own_attitude and self.height is not None:
                    frame = self._own_frame(self.believed[stamp])      # (as the pair last saw it)
                if self.own_attitude and self.height is None:
                    continue                   # nothing can be placed before the road has been seen once
                if cam["role"] != "stereo":
                    rng_img = self._mono_range(cam, out[::-1], *frame)
                if rng_img is None:
                    continue
                scan, xy, segs, kerb = self._camera(cam, image, rng_img, *frame, labels=seen)
                # (each answer stands for the time since the one before it from the same network)
                dt = (self.stereo_every if cam["role"] == "stereo" else self.mono_every) * PERCEPTION_DT
                scans.append((dt,) + scan)
                dets += [tuple(seg) + (dt,) for seg in segs]
                paint.append(xy)
                self.kerbs.append((dt * (self.SCENE_MONO if cam["role"] == "mono" else 1), kerb))
        if self.lidar is not None:
            buf = self.lidar["sensor"].GetMostRecentDIBuffer()
            stamp = round(buf.TimeStamp, 4) if buf.HasData() else -1.0
            if stamp != self.lidar["stamp"] and stamp in self.frames and not (self.own_attitude and self.height is None):
                self.lidar["stamp"] = stamp
                scans.append((PERCEPTION_DT,) + self._scanner(
                    self.lidar, buf.GetDIData()[..., 0], *self.frames[stamp], self.LIDAR_RANGE, 0.02, 0.01,
                    int(round(math.degrees(self.LIDAR_HFOV) / 0.5)), 0.5 * self.LIDAR_HFOV, self.Z_LIDAR))
        if paint:
            self.paint = np.concatenate(paint)
        return scans, dets

    def _stereo_rows(self, disp):
        """The range image from the disparity of the rows that were sent to the network (top row
        first, as it comes back). The rows that were not sent have no range."""
        (r0, r1), full = self.rows, np.zeros((self.CAM_H, self.CAM_W), dtype=np.float32)
        full[r0:r1] = disp
        r = self._stereo_range(full[::-1])
        r[:(self.CAM_H - r1) // 2] = r[(self.CAM_H - r0) // 2:] = 0.0
        return r

    def _stereo_range(self, disp):
        """The range along each ray of the half-size depth image, from the disparity of the left
        image (bottom row first). 0 where there is no telling: at depth edges, where a matcher
        puts pixels between the near and the far surface, and at the left rim, which the right
        camera does not see."""
        d = 0.25 * (disp[0::2, 0::2] + disp[1::2, 0::2] + disp[0::2, 1::2] + disp[1::2, 1::2])
        tol = 1.0 + 0.1 * d
        edge = np.zeros(d.shape, dtype=bool)
        jump = np.abs(np.diff(d, axis=1)) > np.minimum(tol[:, 1:], tol[:, :-1])
        edge[:, 1:] |= jump
        edge[:, :-1] |= jump
        jump = np.abs(np.diff(d, axis=0)) > np.minimum(tol[1:], tol[:-1])
        edge[1:] |= jump
        edge[:-1] |= jump
        unseen = 2.0 * np.arange(d.shape[1])[None, :] < d
        r = self.CAM_F * self.BASELINE / np.maximum(d, 1e-3) / self.rays[..., 0]
        r = np.minimum(r, self.CAM_FAR)
        r[edge | unseen] = 0.0
        return r.astype(np.float32)

    def _mono_range(self, cam, inv, ref_p, ref_R):
        """The range along each ray of the half-size depth image, from the relative inverse depth
        a monocular network gives for the image (bottom row first). That output has neither scale
        nor offset. Both are fitted so that the pixels showing the ground come out where the
        ground is, which the camera's height and attitude give. None if the fit fails."""
        inv = 0.25 * (inv[0::2, 0::2] + inv[1::2, 0::2] + inv[0::2, 1::2] + inv[1::2, 1::2])
        p, R = ref_p + ref_R @ cam["pos"], ref_R @ cam["R"]
        down = -(self.rays @ R.T.astype(np.float32))[..., 2]
        axial = self.rays[..., 0]                               # depth along the optical axis per unit of range
        zg = np.where(down > 0.03, p[2] / np.maximum(down, 0.03) * axial, np.inf)      # depth of the ground per pixel
        m = np.zeros(inv.shape, dtype=bool)
        m[:inv.shape[0] // 4] = True                            # to begin with: the bottom of the image
        m &= np.isfinite(zg)
        z = None
        for _ in range(4):
            x, y = inv[m].astype(np.float64), 1.0 / zg[m]
            if len(x) < 1500 or np.ptp(x) < 1e-6:           # too little ground in view, or a blank answer
                return None
            a, b = np.polyfit(x, y, 1)
            if a <= 0.0:
                return None
            z = 1.0 / np.maximum(a * inv + b, 1.0 / self.CAM_FAR)
            m = np.isfinite(zg) & (zg < 8.0) & (np.abs(z - zg) < 0.06 * zg + 0.05)
        return np.minimum(z / axial, self.CAM_FAR).astype(np.float32) if m.sum() >= 1500 else None

    def _keep(self, dev, P, valid, ref_p, ref_R, **raw):
        """For the viewer: the data of a sensor as it came in, and its points in the chassis frame."""
        dev.update(raw)
        dev["cloud"] = (P[valid] - ref_p.astype(np.float32)) @ ref_R.astype(np.float32)

    def _is_own(self, P, ref_p, ref_R, grow):
        loc = (P - ref_p.astype(np.float32)) @ ref_R.astype(np.float32)
        x0, x1, hw = self.own
        return (loc[..., 0] > x0 - grow) & (loc[..., 0] < x1 + grow) & (np.abs(loc[..., 1]) < hw + grow)

    def _paint(self, image, flat, p, R, head, reach, labels=None):
        """Ground cells that look painted. The image is laid out on the ground plane, as a picture
        from above with 5 cm cells, in linear light. A cell is paint if it is 1.8 times lighter
        than the ground 20 cm to both sides of it, in one of four directions, and the depth image
        ('flat', at half the size) says that both of those are ground. That holds for a stripe in
        the sun and for one in the shade. It does not hold for the edge of a shadow, which is
        lighter than one side only, for anything wide, or for the light sill of a car, which has
        the car on one side.

        Paint that is nearly worn away is not 1.8 times lighter than the road. Where the scene
        network sees a marking ('labels', the size of the image), and 10 cm around that, 1.2 times
        is enough. The network alone would not do: far away it smears a line over the cells
        next to it. The two together find faint paint about five times as often as the rule
        alone, with hardly more false cells (docs/sensors.md).

        Returns the world position of the cells and the pixel each was read from."""
        c, n = 0.05, int(reach / 0.05)
        gx, gy = (np.arange(n) + 0.5) * c, (np.arange(2 * n) + 0.5 - n) * c
        ch, sh = math.cos(head), math.sin(head)
        X = p[0] + gx[:, None] * ch - gy[None, :] * sh
        Y = p[1] + gx[:, None] * sh + gy[None, :] * ch
        d = np.stack([X - p[0], Y - p[1], np.full_like(X, -p[2])], axis=-1) @ R       # into the camera frame
        fwd = np.maximum(d[..., 0], 1e-3)
        u = np.rint(0.5 * self.CAM_W - self.CAM_F * d[..., 1] / fwd - 0.5).astype(np.int64)
        v = np.rint(0.5 * self.CAM_H + self.CAM_F * d[..., 2] / fwd - 0.5).astype(np.int64)
        ok = (d[..., 0] > 0.2) & (u >= 0) & (u < self.CAM_W) & (v >= 0) & (v < self.CAM_H) & \
            (gx[:, None] ** 2 + gy[None, :] ** 2 < reach * reach)
        u, v = np.clip(u, 0, self.CAM_W - 1), np.clip(v, 0, self.CAM_H - 1)
        B = np.where(ok, self.linear[image[v, u].max(axis=-1)], np.nan)
        P = np.pad(B, 4, constant_values=np.nan)
        G = np.pad(ok & flat[v // 2, u // 2], 4)
        s = lambda A, i, j: A[4 + i:4 + i + B.shape[0], 4 + j:4 + j + B.shape[1]]
        ridge = np.zeros(B.shape, dtype=bool)
        hint = None
        if labels is not None:
            hint = _grown((labels[v, u] == MARKING) & G[4:-4, 4:-4], 2)
            faint = np.zeros(B.shape, dtype=bool)
        with np.errstate(invalid="ignore"):
            for i, j in ((4, 0), (0, 4), (3, 3), (3, -3)):
                sides, ground = np.maximum(s(P, i, j), s(P, -i, -j)), s(G, i, j) & s(G, -i, -j)
                ridge |= (B > 1.8 * sides + 0.004) & ground
                if hint is not None:
                    faint |= (B > self.PAINT_HINT * sides + 0.004) & ground
        if hint is not None:
            ridge |= faint & hint
        ii, jj = np.nonzero(ridge)
        return np.stack([X[ii, jj], Y[ii, jj]], axis=1), v[ii, jj], u[ii, jj]

    def _camera(self, cam, image, depth, ref_p, ref_R, labels=None):
        """One camera with its depth image: obstacles and free ground from the depth, painted
        lines from the image. image is (h, w, 3) uint8, depth (h/2, w/2) the range along each
        pixel's ray (0 where unknown), both with the bottom row first. The chassis frame is the
        one at the time the image was taken. labels is what the scene network makes of the image
        (h, w), if it runs: it points out faint paint, the kerbs, and the car's own body.
        Returns the planar scan, the cells of paint, the line pieces among them, and the points
        of kerb."""
        p, R = ref_p + ref_R @ cam["pos"], (ref_R @ cam["R"]).astype(np.float32)
        e0, e1, e2 = cam["err"]
        err = lambda r: e0 + e1 * r + e2 * r * r          # the range error the processing assumes
        # The map takes obstacles only from as far as they can be placed to RANGE_TOL, and free
        # ground only from as far as ground can be told from a kerb, which needs the height of a
        # point to HEIGHT_TOL. Ground seen further out, up to where even the top of a kerb would
        # pass for ground, is reported as probably free. Monocular depth never tells a kerb from
        # the ground, so all the ground it shows is only probably free.
        rr = np.linspace(0.3, self.CAM_RANGE, 235)
        placed = err(rr) <= self.RANGE_TOL
        level = err(rr) * p[2] / rr
        reach_hit = float(rr[placed].max()) if placed.any() else 0.0
        reach = float(rr[placed & (level <= self.HEIGHT_TOL)].max()) if (placed & (level <= self.HEIGHT_TOL)).any() else 0.0
        reach_far = float(rr[level <= 1.65 * self.HEIGHT_TOL].max()) if (level <= 1.65 * self.HEIGHT_TOL).any() else 0.0
        if cam["role"] == "mono":
            reach_far = self.MONO_RANGE
        dw = self.rays @ R.T
        P = p.astype(np.float32) + depth[..., None] * dw
        z = P[..., 2]
        zs = err(depth) * np.abs(dw[..., 2])              # height error that the range error causes
        own = self._is_own(P, ref_p, ref_R, 0.15)
        seen = (depth > 0.3) & (depth < 28.0) & (z < self.Z_TOP) & ~own
        kerb = np.zeros((0, 2))
        if labels is not None:
            small = labels[::2, ::2]
            # Along the edge of the car's own bonnet in the image, a depth network puts pixels
            # somewhere between the bonnet and what lies behind it: points in the air next to
            # the car, which the outline above does not cover. Within 3 pixels of what the
            # network calls the car itself, a point less than 0.6 m from the car is taken only
            # if it is ground. (Not every point in that band: the road 4 to 5 m ahead lies in
            # it too, and that is where the pair sees a kerb as an obstacle.)
            beside = _grown(small == OWN, 3) & self._is_own(P, ref_p, ref_R, 0.6)
            seen &= ~beside | (np.abs(z) < self.Z_GROUND + 1.25 * zs)
            # A kerb is too low to be told from the ground by its height beyond 8 m, and a single
            # camera cannot tell it at all. The network knows one when it sees it.
            there = seen & (small == KERB) & (depth < self.KERB_RANGE[cam["role"]]) & (z > -0.15 - 2.5 * zs) & \
                (z < 0.30 + 2.5 * zs)
            kerb = P[there][:, :2].astype(float)
        if self.show:
            self._keep(cam, P[::2, ::2], ((depth > 0.3) & (depth < 0.99 * self.CAM_FAR))[::2, ::2], ref_p, ref_R,
                       rgb=image, range=np.where(depth < 0.99 * self.CAM_FAR, depth, 0.0))
        ground = seen & (np.abs(z) < self.Z_GROUND + 1.25 * zs)
        obstacle = seen & (z > self.Z_OBSTACLE + 2.5 * zs)
        head = math.atan2(R[1, 0], R[0, 0])
        half = cam["half"]
        n_bins = int(round(math.degrees(2.0 * half) / 0.5))
        ang, r_hit, r_free, r_stop, _ = planar_scan(P, p, obstacle, ground, seen & ~ground, n_bins, half, head,
                                                    max(reach_hit, 0.5))
        r_free = np.minimum(r_free, reach)
        r_far = r_free
        if reach_far > reach + 0.5:
            maybe = seen & (np.abs(z) < self.Z_GROUND + 1.5 * zs)
            r_far = np.maximum(r_free, planar_scan(P, p, maybe & False, maybe, seen & ~maybe, n_bins, half, head,
                                                   reach_far)[2])
            r_far = np.where(np.isfinite(r_stop), r_free, r_far)      # not past something that stands up

        # paint: found in the image as laid out on the ground, then checked against the depth
        xy, vv, uu = self._paint(image, ground, p, R, head, cam["paint"], labels)
        t = np.sqrt((xy[:, 0] - p[0]) ** 2 + (xy[:, 1] - p[1]) ** 2 + p[2] ** 2)      # range of the ground cell
        vd, ud = np.minimum(vv // 2, depth.shape[0] - 1), np.minimum(uu // 2, depth.shape[1] - 1)
        # the depth has to agree that the pixel is on the ground: a white car is not, nor is the
        # top of a kerb
        keep = ground[vd, ud] & (np.abs(depth[vd, ud] - t) < 0.10 + 0.035 * t + 2.5 * err(t))
        keep[keep] = ~self._is_own(np.concatenate([xy[keep], np.zeros((keep.sum(), 1))], axis=1).astype(np.float32),
                                   np.array([ref_p[0], ref_p[1], 0.0]), ref_R, 0.3)
        # and nothing past the first thing that stands up from the ground on its bearing is a line
        rho = np.hypot(xy[:, 0] - p[0], xy[:, 1] - p[1])
        rel = (np.arctan2(xy[:, 1] - p[1], xy[:, 0] - p[0]) - head + math.pi) % (2.0 * math.pi) - math.pi
        b = np.clip(np.floor((rel + half) / (2.0 * half) * n_bins).astype(int), 0, n_bins - 1)
        keep &= ~(rho > r_stop[b] + 0.1)
        xy = xy[keep]
        # a small picture of what the camera is read as, for the viewer (top row first):
        # 0 nothing, 1 ground, 2 obstacle, 3 the car itself, 4 unclear, 5 paint
        view = np.where(own & (depth > 0.0), 3, np.where(obstacle, 2, np.where(ground, 1, np.where(seen, 4, 0)))).astype(np.uint8)
        view = view[::-5, ::5].copy()
        view[np.minimum((depth.shape[0] - 1 - vd[keep]) // 5, view.shape[0] - 1), np.minimum(ud[keep] // 5, view.shape[1] - 1)] = 5
        cam["view"] = view
        # (a single camera places a stripe by the ground alone, so only a longer piece counts)
        segs = paint_segments(xy, p[:2], min_count=1, min_len=0.35 if cam["role"] == "stereo" else 0.6)
        return (p[:2], ang, r_hit, r_free, r_far, r_stop), xy, segs, kerb

    def _scanner(self, dev, r, ref_p, ref_R, r_max, sigma, dropout, n_bins, half_fov, z_min=0.0):
        """A lidar: a range per beam (0 where nothing came back).
        z_min: returns from lower than this end the free part of a ray but are not obstacles. The
        beams of a lidar are far apart on the ground, so a kerb is hit somewhere on its top, not at
        its face, and placing those returns in the map blurs where the kerb is. The cameras see it."""
        k = self.noise
        p, R = ref_p + ref_R @ dev["pos"], (ref_R @ dev.get("R", np.eye(3))).astype(np.float32)
        r = np.array(r, dtype=np.float32)
        if k > 0.0:
            r = r + self.rng.normal(0.0, sigma * k, r.shape).astype(np.float32) * (r > 0.0)
            r[self.rng.random(r.shape) < dropout * k] = 0.0
        P = p.astype(np.float32) + r[..., None] * (dev["rays"] @ R.T)
        z = P[..., 2]
        seen = (r > 0.3) & (r < r_max) & (z < self.Z_TOP) & ~self._is_own(P, ref_p, ref_R, 0.15)
        if self.show:
            self._keep(dev, P, (r > 0.3) & (r < 1.4 * r_max), ref_p, ref_R, range=r)
        ground = seen & (np.abs(z) < self.Z_GROUND + 1.5 * sigma * max(k, 0.3))
        obstacle = seen & (z > max(z_min, self.Z_OBSTACLE + 2.5 * sigma * max(k, 0.3)))
        head = math.atan2(R[1, 0], R[0, 0])
        ang, r_hit, r_free, r_stop, before = planar_scan(P, p, obstacle, ground, seen & ~ground, n_bins,
                                                         half_fov, head, r_max)
        if z_min > 0.0:
            # Where the first thing a ray meets is low, the beam that found it came in over its
            # edge: the ground is free only as far as it was seen in front of it. Otherwise the
            # lidar would keep clearing the face of a kerb that the cameras put in the map.
            low = np.isfinite(r_stop) & ~(r_hit < r_stop + 0.3)
            r_free = np.where(low, np.minimum(r_free, before), r_free)
        self.fans.append((p[:2], head, half_fov, r_max))
        return p[:2], ang, r_hit, r_free
