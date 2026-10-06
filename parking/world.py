"""The Chrono world: the sedan, the terrain and its surfaces, painted lines, kerbs, parked cars, light."""

import math
import os
import struct
import tempfile
import zlib

import numpy as np

from .chrono_env import chrono, sens, veh
from .config import STEP
from .ground import Ground
from .paint import lay, paint_textures
from .scenario import parked_model
from .vehicle import EGO


def write_png(path, rgb):
    """A (rows, columns, 3) uint8 image as a PNG file."""
    h, w, _ = rgb.shape
    raw = np.concatenate([np.zeros((h, 1), np.uint8), rgb.reshape(h, 3 * w)], axis=1).tobytes()

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xffffffff)

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) +
                chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def surface_textures():
    """Image files for the surfaces of the lot: worn asphalt, concrete, grass. They are made here,
    once, from band-limited noise, so that the road has the grain, the blotches and the cracks
    that a camera sees on a real one. A stereo matcher needs that: on a road of one flat colour
    there is nothing for it to match."""
    folder = os.path.join(tempfile.gettempdir(), "chrono_parking_surfaces_1")
    files = {name: os.path.join(folder, name + ".png") for name in ("asphalt", "concrete", "grass")}
    if all(os.path.exists(f) for f in files.values()):
        return files
    os.makedirs(folder, exist_ok=True)
    rng = np.random.default_rng(7)

    def noise(n, lo, hi):         # seamless on an n x n tile, wavelengths lo to hi texels, unit variance
        f = np.fft.fftfreq(n)
        r = np.hypot(f[:, None], f[None, :])
        x = np.fft.ifft2((rng.normal(size=(n, n)) + 1j * rng.normal(size=(n, n))) * ((r >= 1.0 / hi) & (r <= 1.0 / lo))).real
        return (x - x.mean()) / x.std()

    def srgb(lin):                # the files hold display values, the renderer turns them back into reflectance
        lin = np.clip(lin, 0.0, 1.0)
        return (255.0 * np.where(lin <= 0.0031308, 12.92 * lin, 1.055 * lin ** (1.0 / 2.4) - 0.055) + 0.5).astype(np.uint8)

    n, t = 1024, SURFACE_TILE / 1024                        # asphalt: reflectance about 0.16
    a = 0.16 + 0.016 * noise(n, 2.0, 6.0) + 0.020 * noise(n, 0.05 / t, 0.40 / t) + 0.018 * noise(n, 0.6 / t, 3.0 / t)
    a += 0.06 * (rng.random((n, n)) < 0.012)                # light stones
    for _ in range(5):                                      # sealed cracks
        q, d = rng.random(2) * n, rng.random() * 2.0 * math.pi
        for _ in range(int(rng.integers(300, 900))):
            d += rng.normal(0.0, 0.12)
            q = (q + (math.cos(d), math.sin(d))) % n
            i, j = int(q[1]), int(q[0])
            a[i, j] = a[i, (j + 1) % n] = a[(i + 1) % n, j] = 0.05
    for _ in range(3):                                      # oil stains
        c, rad = rng.random(2) * n, rng.uniform(0.15, 0.45) / t
        yy, xx = np.ogrid[:n, :n]
        dy, dx = np.minimum(abs(yy - c[1]), n - abs(yy - c[1])), np.minimum(abs(xx - c[0]), n - abs(xx - c[0]))
        a *= 1.0 - 0.45 * np.exp(-(dx * dx + dy * dy) / (2.0 * rad * rad))
    g = srgb(a)
    write_png(files["asphalt"], np.stack([g, g, np.minimum(g.astype(int) + 3, 255).astype(np.uint8)], axis=-1))
    n, t = 512, 0.5 * SURFACE_TILE / 512
    g = srgb(0.42 + 0.03 * noise(n, 2.0, 8.0) + 0.035 * noise(n, 0.08 / t, 0.6 / t))
    write_png(files["concrete"], np.stack([g, g, (0.97 * g).astype(np.uint8)], axis=-1))
    v = 0.5 * noise(n, 2.0, 5.0) + 0.6 * noise(n, 0.05 / t, 0.5 / t)
    write_png(files["grass"], np.stack([srgb(0.06 + 0.02 * v), srgb(0.14 + 0.045 * v), srgb(0.035 + 0.012 * v)], axis=-1))
    return files


SURFACE_TILE = 6.0             # the asphalt image covers this much road before it repeats [m]

# The skies the sensors can render under: a sky image that ships with Chrono, the azimuth and
# elevation of the sun in that image [deg], and the strength of the sun and of the light from the
# rest of the sky. The sun is a directional light that stands where the image has it.
SKIES = {"clear": ("sensor/textures/sky_2_4k.hdr", 36.0, 40.8, 2.2, 0.14),
         "low": ("sensor/textures/driving_school_4k.hdr", 36.0, 32.4, 2.6, 0.14),
         "overcast": ("sensor/textures/kloppenheim_06_4k.hdr", 39.4, 60.0, 0.45, 0.50)}


def light_scene(scene, sky):
    """Sun, sky light, background and exposure of the scene that the sensors render. Returns the
    exposure and the vignette that are left to be put on the images (1 and 0 with a renderer
    that has these settings: Metal RT has, OptiX and Vulkan RT have not)."""
    image, az, el, sun, ambient = SKIES[sky]
    scene.SetAmbientLight(chrono.ChVector3f(ambient, ambient, 1.08 * ambient))
    scene.AddDirectionalLight(chrono.ChColor(sun, 0.97 * sun, 0.90 * sun), math.radians(el), math.radians(az))
    bg = sens.Background()
    path = chrono.GetChronoDataFile(image)
    if os.path.exists(path):
        bg.mode, bg.env_tex = sens.BackgroundMode_ENVIRONMENT_MAP, path
    else:
        bg.mode = sens.BackgroundMode_GRADIENT
        bg.color_zenith, bg.color_horizon = chrono.ChVector3f(0.30, 0.45, 0.80), chrono.ChVector3f(0.70, 0.78, 0.88)
    scene.SetBackground(bg)
    # One fixed exposure, set for the road as a camera's auto-exposure would settle: sunlit asphalt
    # comes out at 120 of 255. The backend has no auto-exposure, so shade is dark and white cars
    # in a low sun burn out. The lens darkens the corners by a quarter.
    road = 0.16 * (sun * math.sin(math.radians(el)) ** 2 + 1.5 * ambient)
    exposure, vignette = (120.0 / 255.0) ** 2.2 / road, 0.12
    if not hasattr(scene, "SetExposure"):
        return exposure, vignette
    scene.SetExposure(exposure)
    scene.SetVignette(vignette)
    return 1.0, 0.0


class _Road(veh.ChTerrain):
    """The road as the tires feel it: height, normal and grip straight from the height field.
    Chrono's own answer for a road built as a mesh comes from casting a ray at the triangles,
    and at their edges that is up to a centimetre too high (the collision margin): the wheels
    would run over hundreds of small steps that are not there."""

    def __init__(self, ground, friction):
        super().__init__()
        self.ground, self.friction = ground, friction

    def GetHeight(self, loc):
        return self.ground.at(loc.x, loc.y)[0]

    def GetNormal(self, loc):
        _, sx, sy = self.ground.at(loc.x, loc.y)
        n = 1.0 / math.sqrt(sx * sx + sy * sy + 1.0)
        return chrono.ChVector3d(-sx * n, -sy * n, n)

    def GetPoint(self, loc):                 # (the point of the road under loc: what a tire model stands on)
        return chrono.ChVector3d(loc.x, loc.y, self.ground.at(loc.x, loc.y)[0])

    def GetCoefficientFriction(self, loc):
        return self.friction


class World:
    def __init__(self, scn, visual=True, tire="tmeasy", wear=1.0, bumps=0.0):
        self.scn, self.wear = scn, wear
        self.ground = Ground(scn.bounds, bumps, getattr(scn, "seed", 0))
        self.materials = {}
        x, y, th = scn.start
        sedan = veh.Sedan()
        sedan.SetContactMethod(chrono.ChContactMethod_SMC)
        sedan.SetChassisCollisionType(veh.CollisionType_HULLS)    # so that hitting a parked car is physical
        sedan.SetChassisFixed(False)
        sedan.SetInitPosition(chrono.ChCoordsysd(chrono.ChVector3d(x, y, 0.25 + self.ground.height(x, y)), chrono.QuatFromAngleZ(th)))
        sedan.SetTireType(veh.TireModelType_PAC02 if tire == "pac02" else veh.TireModelType_TMEASY)
        sedan.SetTireStepSize(STEP)
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
        self.inputs = veh.DriverInputs()

        mat = chrono.ChContactMaterialSMC()
        mat.SetFriction(0.9)
        mat.SetRestitution(0.01)
        self.terrain = veh.RigidTerrain(self.system)
        x0, y0, x1, y1 = scn.bounds
        if self.ground.amp > 0.0:          # the road as a mesh over the heights, which Chrono reads from an image
            image, low, high = self.ground.image(write_png)
            patch = self.terrain.AddPatch(mat, chrono.ChCoordsysd(chrono.ChVector3d(*self.ground.center, 0.0), chrono.QUNIT),
                                          image, *self.ground.size, low, high)
        else:
            patch = self.terrain.AddPatch(mat, chrono.ChCoordsysd(
                chrono.ChVector3d(0.5 * (x0 + x1), 0.5 * (y0 + y1), 0.0), chrono.QUNIT),
                (x1 - x0) + 60.0, (y1 - y0) + 60.0)
        patch.SetColor(chrono.ChColor(0.23, 0.23, 0.25))
        self.surfaces = surface_textures() if visual else {}
        if visual:
            patch.SetTexture(self.surfaces["asphalt"], ((x1 - x0) + 60.0) / SURFACE_TILE, ((y1 - y0) + 60.0) / SURFACE_TILE)
        self.terrain.Initialize()
        # what the tires roll on: the flat patch itself, or the height field behind the mesh
        self.road = _Road(self.ground, 0.9) if self.ground.amp > 0.0 else self.terrain

        self.contact_mat = mat
        self._decor(visual)
        self._parked_cars(visual)
        self._actuators()
        # The radius the rear wheels roll on, by the usual rule: two parts the radius of the free
        # tire and one part the height of the axle over the road when the car stands. That is
        # geometry the car knows about itself. (It is 1 percent less than the free radius here.)
        rear = self.car.GetDriveline().GetDrivenAxleIndexes()
        self.rear = 1 if 0 in list(rear) else 0
        stand = np.mean([self.car.GetSpindlePos(self.rear, s).z - self.ground.height(self.car.GetSpindlePos(self.rear, s).x,
                                                                                  self.car.GetSpindlePos(self.rear, s).y)
                         for s in (veh.LEFT, veh.RIGHT)])
        self.roll_radius = (2.0 * EGO.wheel_radius + float(stand)) / 3.0

    # ---- actuators: the car is driven by physical commands, not by pedal positions -------------

    def _actuators(self):
        """Set the car up for direct actuation and learn the actuator limits from the model.

        Drive torque is applied to the half-shafts of the driven axle, with the gearbox in
        neutral so that the engine is out of the loop. The steering command is a road-wheel
        angle; at power-up the steering is run through its travel once to read which rack
        position gives which angle, and the brakes are applied fully to read their capacity."""
        car = self.car
        car.GetTransmission().asAutomatic().SetDriveMode(veh.ChAutomaticTransmission.DriveMode_NEUTRAL)
        self.driven = [car.GetSuspension(int(i)) for i in car.GetDriveline().GetDrivenAxleIndexes()]
        self.brakes = [car.GetBrake(a, side) for a in range(car.GetNumberAxles()) for side in (veh.LEFT, veh.RIGHT)]
        racks, angles = [0.0], [0.0]
        for rack in (0.2, 0.4, 0.6, 0.8, 1.0, 0.5, 0.0):
            for _ in range(int(0.15 / STEP)):
                self._advance(rack, 1.0, 0.0)
            if rack > racks[-1]:
                racks.append(rack)
                angles.append(self.steer_angle())
        self.rack_of_angle = (np.array(angles), np.array(racks))
        EGO.steer_max = angles[-1]
        EGO.brake_torque_max = sum(b.GetBrakeTorque() for b in self.brakes)

    def steer_angle(self):
        """Measured road-wheel steering angle: mean of the two front wheels [rad]."""
        return 0.5 * (self.car.GetSteeringAngle(0, veh.LEFT) + self.car.GetSteeringAngle(0, veh.RIGHT))

    def _advance(self, rack, braking, drive_torque):
        t = self.system.GetChTime()
        self.inputs.m_steering, self.inputs.m_throttle, self.inputs.m_braking = rack, 0.0, braking
        self.terrain.Synchronize(t)
        self.sedan.Synchronize(t, self.inputs, self.road)
        for axle in self.driven:                 # Chrono's half-shafts turn opposite to the wheels
            for side in (veh.LEFT, veh.RIGHT):
                axle.ApplyAxleTorque(side, -drive_torque / (2 * len(self.driven)))
        self.terrain.Advance(STEP)
        self.sedan.Advance(STEP)

    def step(self, steer_angle, drive_torque, brake_torque):
        """Advance one step under physical commands: road-wheel steering angle [rad], total drive
        torque at the wheels [N m, negative drives backwards], total brake torque [N m]."""
        rack = math.copysign(float(np.interp(abs(steer_angle), *self.rack_of_angle)), steer_angle)
        self._advance(rack, min(max(brake_torque / EGO.brake_torque_max, 0.0), 1.0), drive_torque)

    def _box(self, body, lx, ly, lz, pos, yaw, rgb, surface=None):
        shape = chrono.ChVisualShapeBox(lx, ly, lz)
        if surface is None:
            shape.SetColor(chrono.ChColor(*rgb))
        elif surface in self.surfaces:       # one of the ground images, repeated over the box
            mat = chrono.ChVisualMaterial()
            mat.SetKdTexture(self.surfaces[surface])
            mat.SetTextureScale(max(lx / (0.5 * SURFACE_TILE), 0.2), max(ly / (0.5 * SURFACE_TILE), 0.2))
            mat.SetRoughness(0.9)
            shape.AddMaterial(mat)
        else:                                # an image file that covers the box once (one material per image)
            if surface not in self.materials:
                mat = chrono.ChVisualMaterial()
                mat.SetKdTexture(surface)
                mat.SetRoughness(0.9)
                self.materials[surface] = mat
            shape.AddMaterial(self.materials[surface])
        body.AddVisualShape(shape, chrono.ChFramed(chrono.ChVector3d(*pos), chrono.QuatFromAngleZ(yaw)))

    def _decor(self, visual):
        if not visual:
            return
        body = chrono.ChBody()
        body.SetFixed(True)
        dip = self.ground.amp
        if self.wear > 0.0:
            # worn paint: patchy, ragged at the edges, some lines faded or nearly gone (paint.py)
            images = paint_textures(write_png)
            for px, py, yaw, length, wide, colour, state, level, variant in lay(self.scn.lines, self.wear, getattr(self.scn, "seed", 0)):
                self._box(body, length, wide, 0.004, (px, py, self.ground.height(px, py) + (0.005 if dip else 0.003)), yaw,
                          (1.0, 1.0, 1.0), images[(colour, state, level, variant)])
        else:
            # paint as it was before it could wear: half-metre pieces, each one flat grey and a
            # little narrower than new, a few gone
            rng = np.random.default_rng(1000 + getattr(self.scn, "seed", 0))
            for x1, y1, x2, y2, color in self.scn.lines:
                length, yaw = math.hypot(x2 - x1, y2 - y1), math.atan2(y2 - y1, x2 - x1)
                n = max(1, int(round(length / 0.5)))
                for k in range(n):
                    v, wide, gone = rng.uniform(0.50, 0.80), 0.12 * rng.uniform(0.9, 1.0), rng.random() < 0.04
                    if gone:
                        continue
                    f = (k + 0.5) / n
                    px, py = x1 + f * (x2 - x1), y1 + f * (y2 - y1)
                    # (on an uneven road a piece lies on the surface under its middle, a little proud of it)
                    pz = 0.003 if dip == 0.0 else self.ground.height(px, py) + 0.005
                    self._box(body, length / n + 0.004, wide, 0.004, (px, py, pz), yaw,
                              (v, v, v) if color == "white" else (v, 0.80 * v, 0.10 * v))
        # (kerbs and the ground beside the lot keep their tops and reach down below the lowest road)
        for cx, cy, lx, ly in self.scn.curbs:
            self._box(body, lx, ly, 0.15 + dip, (cx, cy, 0.075 - 0.5 * dip), 0.0, (0.66, 0.66, 0.64), "concrete")
        for cx, cy, lx, ly, rgb in self.scn.pads:
            self._box(body, lx, ly, 0.14 + dip, (cx, cy, 0.07 - 0.5 * dip), 0.0, rgb, "grass" if rgb[1] > rgb[0] + 0.05 else "concrete")
        self.system.Add(body)

    def _parked_cars(self, visual):
        data = chrono.GetChronoDataPath() + "vehicle/"
        meshes = {}                # each mesh file is loaded once and shared by the cars that use it
        for car in self.scn.cars:
            m = parked_model(car["model"])
            half = 0.5 * (m["x1"] - m["x0"])
            mid = 0.5 * (m["x1"] + m["x0"])
            c, s = math.cos(car["yaw"]), math.sin(car["yaw"])
            body = chrono.ChBody()
            body.SetFixed(True)
            body.SetPos(chrono.ChVector3d(car["cx"] - mid * c, car["cy"] - mid * s, m["z"] + self.ground.height(car["cx"], car["cy"])))
            body.SetRot(chrono.QuatFromAngleZ(car["yaw"]))
            hull = chrono.vector_ChVector3d()        # the body outline, from 0.1 m below to 1.1 m above the mesh origin
            for z in (-0.1, 1.1):
                for x, y in m["outline"]:
                    hull.push_back(chrono.ChVector3d(float(x), float(y), z))
            body.AddCollisionShape(chrono.ChCollisionShapeConvexHull(self.contact_mat, hull), chrono.ChFramed())
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
                        if f not in meshes:
                            meshes[f] = chrono.ChTriangleMeshConnected.CreateFromWavefrontFile(f, True, True)
                        shape = chrono.ChVisualShapeTriangleMesh(meshes[f], True)    # keeps the .mtl colours
                        body.AddVisualShape(shape, chrono.ChFramed(chrono.ChVector3d(*pos),
                                                                   chrono.QuatFromAngleZ(yaw)))
                else:
                    self._box(body, 2.0 * half, 2.0 * m["hw"], 1.3, (mid, 0.0, 0.5), 0.0, (0.5, 0.5, 0.55))
            self.system.Add(body)

    def wheel_travel(self):
        """How far the axle that is not driven has rolled since the start [m], forwards positive:
        what a wheel encoder counts."""
        turn = np.mean([self.car.GetSuspension(self.rear).GetAxle(s).GetPos() for s in (veh.LEFT, veh.RIGHT)])
        return -float(turn) * self.roll_radius

    def state(self):
        """Rear-axle pose (x, y, heading) and signed forward speed."""
        frame = self.car.GetChassisBody().GetFrameRefToAbs()
        p = frame.GetPos()
        ax = frame.GetRotMat().GetAxisX()
        th = math.atan2(ax.y, ax.x)
        return (p.x - EGO.ref_to_rear * math.cos(th), p.y - EGO.ref_to_rear * math.sin(th), th), self.car.GetSpeed()
