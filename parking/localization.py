"""Where the car thinks it is.

Everything the agent does goes by an estimate of its pose: where it puts what the sensors show, the
plan, the steering. The true pose is for the physics and the score, unless the car is told it
(--give pose). How it leans and how high it rides it has to see for itself (SensorRig._road_plane),
and its speed it counts off its wheels."""

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
                While the car is told to hold (`hold`), it carries the position it has on by
                its wheels, along the heading the receiver gives, and leaves the receiver's
                position aside. Over the 20 m and half a minute of a parking maneuver the
                wheels are good to a few centimetres, and the receiver's position wanders by a
                decimetre or two. A map made a few seconds ago then stays where the car is,
                which is what parking needs. The heading stays the receiver's: it is good to
                0.3 degrees however far the car turns, where a gyro with a scale error of 1
                percent is 0.9 degrees off after the turn into a stall.
    'odometry'  No reception: the car counts how far its wheels have rolled and adds up its yaw
                rate. The rolling radius is known to 0.5 percent and the yaw rate to 1 percent
                (one value each for the run), the gyro has 0.02 deg/s of bias left after being
                zeroed at standstill, and sideways slip is not measured. The error grows with
                the distance and the time. The estimate starts at the true pose.

    With either, the pitch and the roll are off by 0.15 degrees for the run (how well the cameras
    are levelled to the body) plus 0.1 degrees that wander with a time constant of a second.

    All of these are multiplied by `scale`. At scale 0 the car knows its true pose.

    The speed comes from the wheels in either case: an encoder on the axle that is not driven,
    which gives a count every 2.2 cm of travel (a 48-tooth ring read on both edges), with a
    rolling radius known to 0.5 percent. The speed is one count over the time since the count
    before, so it is fine at driving speed and coarse at a crawl, and when the car stops the
    reading only falls as fast as the wait for the next count allows. With `true_speed` the car
    is told its speed."""

    SOURCES = ("gps", "odometry")
    COUNT = 2.0 * math.pi * 0.33 / 96.0          # travel per encoder count [m]

    def __init__(self, pose, rng, source="gps", scale=1.0, true_speed=False):
        self.source, self.scale, self.rng = source, scale, rng
        self.true, self.t = tuple(pose), None
        self.true_speed = true_speed
        self.k_wheel = 1.0 + 0.005 * float(rng.normal())
        self.rolled, self.counts, self.t_count, self.v_wheel = 0.0, 0, None, 0.0
        self.level = np.radians(0.15) * scale * rng.normal(size=2)          # pitch, roll: the constant part
        self.wander = np.zeros(2)
        self.held = False
        if source == "gps":
            self.sigma = np.array([0.10, 0.10, math.radians(0.3)]) * scale     # x, y, heading
            self.tau = np.array([30.0, 30.0, 20.0])
            self.drive = self.sigma * rng.normal(size=3)        # what the error is heading for
            self.error = self.drive.copy()                      # the error itself, which follows smoothly
            # (the wheels, for when the position is held: from a number of their own, so that
            # the receiver's error is the same run with or without holding)
            own = np.random.default_rng(rng.bit_generator.seed_seq.spawn(1)[0])
            self.k_dist = 1.0 + 0.005 * scale * float(own.normal())
        else:
            self.k_dist = 1.0 + 0.005 * scale * float(rng.normal())
            self.k_yaw = 1.0 + 0.01 * scale * float(rng.normal())
            self.bias = math.radians(0.02) * scale * float(rng.normal())
        self.pose = self._gps() if source == "gps" and scale else tuple(pose)

    def _gps(self):
        return (self.true[0] + self.error[0], self.true[1] + self.error[1], wrap(self.true[2] + self.error[2]))

    def hold(self, on=True):
        """From now on carry the position on by the wheels (on), or take it from the receiver
        again (off). With 'odometry' the wheels carry it anyway."""
        self.held = bool(on) and self.source == "gps" and self.scale != 0.0

    def update(self, true, t, rolled=None):
        """The estimate after the car has moved to the pose `true` at time t. rolled: how far
        its wheels have rolled since the start (if not given, the travel along the car is used)."""
        dt = 0.0 if self.t is None else t - self.t
        before, self.true, self.t = self.true, tuple(true), t
        self._encoder(before, dt, rolled)
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
            if not self.held:
                self.pose = self._gps()
                return self.pose
            # held: what the wheels have rolled, along the heading the receiver gives
            mid = before[2] + 0.5 * wrap(true[2] - before[2])
            ds = ((true[0] - before[0]) * math.cos(mid) + (true[1] - before[1]) * math.sin(mid)) * self.k_dist
            x, y, th0 = self.pose
            th = wrap(true[2] + self.error[2])
            mid = th0 + 0.5 * wrap(th - th0)
            self.pose = (x + ds * math.cos(mid), y + ds * math.sin(mid), th)
            return self.pose
        dx, dy, dth = true[0] - before[0], true[1] - before[1], wrap(true[2] - before[2])
        mid = before[2] + 0.5 * dth
        ds = (dx * math.cos(mid) + dy * math.sin(mid)) * self.k_dist       # along the car: what the wheels turn
        dth = dth * self.k_yaw + self.bias * dt
        x, y, th = self.pose
        self.pose = (x + ds * math.cos(th + 0.5 * dth), y + ds * math.sin(th + 0.5 * dth), wrap(th + dth))
        return self.pose

    def _encoder(self, before, dt, rolled):
        """Count what the wheels have rolled since the last call, and time the counts."""
        if dt <= 0.0:
            return
        if rolled is None:
            mid = before[2] + 0.5 * wrap(self.true[2] - before[2])
            ds = ((self.true[0] - before[0]) * math.cos(mid) + (self.true[1] - before[1]) * math.sin(mid)) * self.k_wheel
        else:
            ds = rolled * self.k_wheel - self.rolled
        if ds == 0.0:
            ds = 1e-12
        self.rolled += ds
        counts = math.floor(self.rolled / self.COUNT)
        if counts != self.counts:
            # when the last of these counts came, within the step
            edge = counts if ds > 0.0 else counts + 1
            t_edge = self.t - dt * (self.rolled - edge * self.COUNT) / ds
            if self.t_count is not None and t_edge > self.t_count:
                self.v_wheel = (counts - self.counts) * self.COUNT / (t_edge - self.t_count)
            self.counts, self.t_count = counts, t_edge
        elif self.t_count is not None:
            # no count: the car cannot be faster than one count in the time it has waited
            most = self.COUNT / max(self.t - self.t_count, 1e-6)
            if abs(self.v_wheel) > most:
                self.v_wheel = math.copysign(most, self.v_wheel)
            if most < 0.01:
                self.v_wheel = 0.0

    def speed(self, v):
        """The speed the car measures, given the true one."""
        return v if self.true_speed else self.v_wheel

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
