"""Where the car thinks it is.

Everything the agent does goes by an estimate of its pose: where it puts what the sensors show, the
plan, the steering. With --pose-noise 0, which is still the default, that estimate is the true
pose. And with any setting the car is still told more than it could know: its true height above
the road, and a pitch and roll that are the true ones with an error added (SensorRig._believed_frame),
so the dive under braking is known for free. Its speed is the true one as well."""

import math

import numpy as np

from .geometry import wrap


class Localization:
    """The pose the car believes it has, from one of two sources.

    'gps'       A satellite receiver with an inertial unit behind it. The position is off by an
                amount that wanders slowly, as the satellites and the reflections off the cars
                around change: 10 cm per axis, changing over about half a minute, and smooth from
                one moment to the next because the inertial unit bridges the fixes. The heading
                is off by 0.3 degrees, wandering over 20 s. Nothing grows with the distance driven.
    'odometry'  No reception: the car counts how far its wheels have rolled and adds up its yaw
                rate. The rolling radius is known to 0.5 percent and the yaw rate to 1 percent
                (one value each for the run), the gyro has 0.02 deg/s of bias left after being
                zeroed at standstill, and sideways slip is not measured. The error grows with
                the distance and the time. The estimate starts at the true pose.

    With either, the pitch and the roll are off by 0.15 degrees for the run (how well the cameras
    are levelled to the body) plus 0.1 degrees that wander with a time constant of a second.

    All of these are multiplied by `scale`. At scale 0 the car knows its true pose."""

    SOURCES = ("gps", "odometry")

    def __init__(self, pose, rng, source="gps", scale=1.0):
        self.source, self.scale, self.rng = source, scale, rng
        self.true, self.t = tuple(pose), None
        self.level = np.radians(0.15) * scale * rng.normal(size=2)          # pitch, roll: the constant part
        self.wander = np.zeros(2)
        if source == "gps":
            self.sigma = np.array([0.10, 0.10, math.radians(0.3)]) * scale     # x, y, heading
            self.tau = np.array([30.0, 30.0, 20.0])
            self.drive = self.sigma * rng.normal(size=3)        # what the error is heading for
            self.error = self.drive.copy()                      # the error itself, which follows smoothly
            self.k_dist = 1.0
        else:
            self.k_dist = 1.0 + 0.005 * scale * float(rng.normal())
            self.k_yaw = 1.0 + 0.01 * scale * float(rng.normal())
            self.bias = math.radians(0.02) * scale * float(rng.normal())
        self.pose = self._gps() if source == "gps" and scale else tuple(pose)

    def _gps(self):
        return (self.true[0] + self.error[0], self.true[1] + self.error[1], wrap(self.true[2] + self.error[2]))

    def update(self, true, t):
        """The estimate after the car has moved to the pose `true` at time t."""
        dt = 0.0 if self.t is None else t - self.t
        before, self.true, self.t = self.true, tuple(true), t
        if self.scale == 0.0:
            self.pose = self.true
            return self.pose
        if dt > 0.0:
            keep = math.exp(-dt / 1.0)
            self.wander = keep * self.wander + math.radians(0.1) * self.scale * math.sqrt(1.0 - keep * keep) * self.rng.normal(size=2)
        if self.source == "gps":
            if dt > 0.0:
                keep = np.exp(-dt / self.tau)
                self.drive = keep * self.drive + self.sigma * np.sqrt(1.0 - keep * keep) * self.rng.normal(size=3)
                self.error += (self.drive - self.error) * (1.0 - math.exp(-dt / 1.0))
            self.pose = self._gps()
            return self.pose
        dx, dy, dth = true[0] - before[0], true[1] - before[1], wrap(true[2] - before[2])
        mid = before[2] + 0.5 * dth
        ds = (dx * math.cos(mid) + dy * math.sin(mid)) * self.k_dist       # along the car: what the wheels turn
        dth = dth * self.k_yaw + self.bias * dt
        x, y, th = self.pose
        self.pose = (x + ds * math.cos(th + 0.5 * dth), y + ds * math.sin(th + 0.5 * dth), wrap(th + dth))
        return self.pose

    def speed(self, v):
        return v * self.k_dist if self.scale else v

    @property
    def tilt(self):
        """The error of the pitch and of the roll the car assumes [rad], or None if it knows them."""
        return tuple(self.level + self.wander) if self.scale else None

    def as_believed(self, scans, dets):
        """What a sensor measured from where the car is, put where the car thinks it is: the scans
        and line detections of the stand-in perception, which works them out in the true frame."""
        if self.scale == 0.0:
            return scans, dets
        dth = wrap(self.pose[2] - self.true[2])
        c, s = math.cos(dth), math.sin(dth)

        def move(x, y):
            x, y = x - self.true[0], y - self.true[1]
            return self.pose[0] + c * x - s * y, self.pose[1] + s * x + c * y

        scans = [(scan[0], move(*scan[1]), scan[2] + dth) + tuple(scan[3:]) for scan in scans]
        dets = [move(d[0], d[1]) + move(d[2], d[3]) + tuple(d[4:]) for d in dets]
        return scans, dets
