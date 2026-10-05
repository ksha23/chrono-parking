"""The ego vehicle: everything about the car is read from the Chrono model at start-up."""

import math

import numpy as np

from .chrono_env import chrono, veh


class Ego:
    def __init__(self):
        self.wheelbase = self.rear = self.front = self.half_width = None
        self.ref_to_rear = self.length = self.center = self.kappa = self.radius = None
        self.mass = self.wheel_radius = self.steer_max = self.brake_torque_max = None
        self.roof = None       # (x of the rear edge, x of the front edge, height), chassis frame

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
            pts += [(q.x, q.y, q.z) for q in hull.GetPoints()]
        pts = np.array(pts)
        top = pts[pts[:, 2] > pts[:, 2].max() - 0.10]       # where sensors can be mounted
        self.roof = (float(top[:, 0].min()), float(top[:, 0].max()), float(pts[:, 2].max()))
        self.rear = -self.ref_to_rear - pts[:, 0].min()     # rear axle -> rear bumper
        self.front = pts[:, 0].max() + self.ref_to_rear     # rear axle -> front bumper
        self.half_width = float(np.abs(pts[:, 1]).max())
        self.length = self.front + self.rear
        self.center = 0.5 * (self.front - self.rear)       # rear axle -> middle of the body
        # kinematic (bicycle) curvature at the model's declared maximum steering angle: the
        # planner stays within this
        self.kappa = math.tan(car.GetMaxSteeringAngle()) / self.wheelbase
        self.radius = 1.0 / self.kappa
        self.mass = car.GetMass()
        self.wheel_radius = car.GetTire(0, veh.LEFT).GetRadius()

    def torque(self, accel):
        """Wheel torque that gives the car this acceleration on level ground."""
        return self.mass * accel * self.wheel_radius


EGO = Ego()
