"""Path tracking: model predictive control of the steering, with the steering gain identified online."""

import collections
import math

import numpy as np

from .config import A_BRAKE, A_DRIVE_MAX, STEER_RATE
from .geometry import wrap
from .vehicle import EGO


def nnls(E, f, max_iter=200):
    """Lawson-Hanson active-set method: minimise ||E x - f|| subject to x >= 0.
    Returns x and the number of iterations."""
    n = E.shape[1]
    P = np.zeros(n, dtype=bool)                 # the passive set: variables allowed to be positive
    x = np.zeros(n)
    w = E.T @ f
    tol = 1e-10 * E.shape[0] * max(np.abs(E).max() * np.abs(f).max(), 1e-30)
    for it in range(max_iter):
        cand = np.where(P, -np.inf, w)
        j = int(np.argmax(cand))
        if cand[j] <= tol:
            break
        P[j] = True
        while True:
            s = np.zeros(n)
            s[P] = np.linalg.lstsq(E[:, P], f, rcond=None)[0]
            if s[P].min() > 0.0:
                break
            neg = P & (s <= 0.0)                # step towards s until the first variable hits zero
            x = x + np.min(x[neg] / (x[neg] - s[neg])) * (s - x)
            P &= x > 1e-14
            if not P.any():
                s = np.zeros(n)
                break
        x = s
        w = E.T @ (f - E @ x)
    return x, it + 1


class LateralMPC:
    """Linear MPC of the lateral motion along a path, written in travelled distance instead of
    time so that it stays well posed down to walking pace.

    State x = (e, psi): lateral and heading error of the rear axle w.r.t. the path. Over one
    step of length h, driving in direction d = +-1 with curvature kappa where the path has
    curvature kappa_ref, the kinematic bicycle gives (to first order in the errors)

        e'   = e + d h psi + h^2/2 (kappa - kappa_ref)
        psi' = psi + d h (kappa - kappa_ref)

    The decision variables are the N curvatures over the horizon. The cost penalises the
    errors, the deviation from the path curvature and curvature changes; the constraints are
    the steering limit |kappa| <= kappa_max and the steering rate. The Hessian of the condensed
    QP is constant, so it is factorised once; each solve turns the QP into a least-distance
    problem and solves that exactly with a non-negative least squares active-set method."""
    N = 20                   # horizon steps
    H = 0.2                  # step [m]  -> 4 m look-ahead
    Q_E, Q_PSI = 10.0, 6.0   # error weights
    Q_END = 3.0              # extra weight on the last step
    R_K = 1.0                # weight on (kappa - kappa_ref)
    R_DK = 1.0               # weight on curvature changes

    def __init__(self):
        N, h = self.N, self.H
        self.D = np.eye(N) - np.eye(N, k=-1)                 # (D K)_k = K_k - K_{k-1}
        Ac = np.vstack([np.eye(N), self.D])                  # rows: curvature, then curvature change
        G = np.vstack([Ac, -Ac])                             # G K <= [hi; -lo]
        self.qp = {}
        for d in (1, -1):
            A = np.array([[1.0, d * h], [0.0, 1.0]])
            B = np.array([0.5 * h * h, d * h])
            Phi, Gam = np.zeros((2 * N, 2)), np.zeros((2 * N, N))
            Ak = np.eye(2)
            for k in range(N):
                Ak = A @ Ak
                Phi[2 * k:2 * k + 2] = Ak
                for j in range(k + 1):
                    Gam[2 * k:2 * k + 2, j] = np.linalg.matrix_power(A, k - j) @ B
            w = np.tile([float(self.Q_E), float(self.Q_PSI)], N)
            w[-2:] *= self.Q_END
            GQ = Gam.T * w
            Hm = 2.0 * (GQ @ Gam + self.R_K * np.eye(N) + self.R_DK * self.D.T @ self.D)
            Linv = np.linalg.inv(np.linalg.cholesky(Hm))      # Hm = L L^T
            self.qp[d] = dict(Phi=Phi, Gam=Gam, GQ=GQ, Linv=Linv, Gt=-G @ Linv.T, GHinv=G @ Linv.T @ Linv)
        self.iters = 0

    def solve(self, d, e0, psi0, k_prev, k_ref, k_max, dk_max):
        """Returns the curvature sequence and the predicted (e, psi) over the horizon."""
        q, N = self.qp[d], self.N
        x0 = np.array([e0, psi0])
        c = np.zeros(N)
        c[0] = k_prev
        f = 2.0 * (q["GQ"] @ (q["Phi"] @ x0 - q["Gam"] @ k_ref) - self.R_K * k_ref - self.R_DK * self.D.T @ c)
        hi = np.concatenate([np.full(N, k_max), c + dk_max])
        lo = np.concatenate([np.full(N, -k_max), c - dk_max])
        # With y = L^T K + L^-1 f the QP is "minimise |y| subject to Gt y >= ht", which Lawson and
        # Hanson reduce to one non-negative least squares problem
        ht = -(np.concatenate([hi, -lo]) + q["GHinv"] @ f)
        E = np.vstack([q["Gt"].T, ht[None, :]])
        rhs = np.zeros(N + 1)
        rhs[-1] = 1.0
        u, self.iters = nnls(E, rhs)
        r = E @ u - rhs
        if abs(r[-1]) > 1e-12:
            K = q["Linv"].T @ (-r[:-1] / r[-1] - q["Linv"] @ f)
        else:                 # only if the constraints contradict each other: fall back to clipping
            K = -q["Linv"].T @ (q["Linv"] @ f)
        K = np.clip(K, -k_max, k_max)
        pred = (q["Phi"] @ x0 + q["Gam"] @ (K - k_ref)).reshape(N, 2)
        return K, pred


class SteeringGain:
    """Online estimate of the gain g in

        curvature = g * tan(road-wheel steering angle)

    separately for forward and reverse, by recursive least squares. For an ideal bicycle
    g = 1 / wheelbase, which is the starting value. The real car turns less than that (tire slip,
    compliance in the steering, front wheels that do not follow Ackermann geometry), and how much
    less is what this identifies.

    The measurement comes from the car's own track over the last half second. Integrating the
    model along the distance driven gives

        change of heading = g * integral of tan(steering angle) d(distance)

    so the heading change is regressed on that integral. This holds while the steering is
    moving, which it nearly always is. (Yaw rate over speed would be the obvious instantaneous
    measurement, but in this simulation it is too noisy at low speed.)"""
    FORGET = 0.98        # per control tick: the estimate follows a change within about a second
    WINDOW = 25          # control ticks, 0.5 s

    def __init__(self):
        self.g = {1: 1.0 / EGO.wheelbase, -1: 1.0 / EGO.wheelbase}
        self.P = {1: 2.0, -1: 2.0}
        self.track = collections.deque(maxlen=self.WINDOW)     # (x, y, heading, tan(angle), direction)

    def update(self, d, delta, pose):
        self.track.append((pose[0], pose[1], pose[2], math.tan(delta), d))
        if len(self.track) < self.WINDOW or self.track[0][4] != d:
            return
        q = np.array(self.track)
        ds = np.hypot(np.diff(q[:, 0]), np.diff(q[:, 1]))
        dist = ds.sum()
        if dist < 0.25:
            return
        x = float((0.5 * (q[1:, 3] + q[:-1, 3]) * ds).sum() / dist)      # distance average of tan(angle)
        if abs(x) < 0.05:
            return
        kappa = wrap(pose[2] - q[0, 2]) / (d * dist)
        P = self.P[d]
        gain = P * x / (self.FORGET + x * P * x)
        g = self.g[d] + gain * (kappa - self.g[d] * x)
        self.g[d] = min(max(g, 0.4 / EGO.wheelbase), 1.5 / EGO.wheelbase)
        self.P[d] = min((P - gain * x * P) / self.FORGET, 2.0)


class MpcTracker:
    """Follows one path segment at a time: turn the wheels while standing, drive, stop.
    Its outputs are physical: road-wheel steering angle [rad], drive torque at the wheels [N m,
    negative for reverse] and brake torque [N m]."""
    A_DEC = 0.5          # deceleration used to approach the end of a segment [m/s^2]
    A_ACC = 0.7
    V_CREEP = 0.15
    KP, KI = 4.0, 2.0    # speed loop: acceleration per speed error [1/s], and its integral [1/s^2]

    def __init__(self):
        self.seg = None
        self.delta = 0.0                 # commanded road-wheel angle [rad]
        self.phase = "idle"
        self.done = True
        self.err = (0.0, 0.0, 0.0)       # lateral, heading, remaining distance
        self.v_cmd = 0.0
        self.mpc = LateralMPC()
        self.gain = SteeringGain()
        self.horizon = np.zeros((0, 2))  # predicted rear-axle positions, for display
        self.k_plan = np.zeros(0)        # planned curvatures over the horizon
        self.k_ref = np.zeros(0)

    def start(self, seg, t, presteer=True):
        self.seg, self.i, self.done = seg, 0, False
        self.phase = "steer" if presteer else "go"
        self.t_phase, self.integ, self.v_cmd, self.t_still = t, 0.0, 0.0, None

    def stop(self):
        if self.phase in ("steer", "go"):
            self.phase, self.t_still = "stop", None

    def update(self, pose, v, t, dt):
        """Returns (steering angle, drive torque, brake torque)."""
        seg = self.seg
        x, y, th = pose
        i1 = min(len(seg.x), self.i + 40)
        self.i += int(np.argmin((seg.x[self.i:i1] - x) ** 2 + (seg.y[self.i:i1] - y) ** 2))
        i, d = self.i, seg.dir
        # the reference is the path between its samples, not the nearest sample: otherwise the
        # reference heading jumps every 10 cm and the steering follows it
        a = max(min(i, len(seg.x) - 2), 0)
        if a > 0 and (x - seg.x[a]) * (seg.x[a + 1] - seg.x[a]) + (y - seg.y[a]) * (seg.y[a + 1] - seg.y[a]) < 0.0:
            a -= 1
        ux, uy = seg.x[a + 1] - seg.x[a], seg.y[a + 1] - seg.y[a]
        f = min(max(((x - seg.x[a]) * ux + (y - seg.y[a]) * uy) / max(ux * ux + uy * uy, 1e-12), 0.0), 1.0)
        th_ref = seg.th[a] + f * wrap(seg.th[a + 1] - seg.th[a])
        s_ref = seg.s[a] + f * (seg.s[a + 1] - seg.s[a])
        e = -math.sin(th_ref) * (x - seg.x[a] - f * ux) + math.cos(th_ref) * (y - seg.y[a] - f * uy)
        psi = wrap(th - th_ref)
        s_rem = seg.length - s_ref
        if s_rem < 1.5:        # near the end, measure what is left along the final heading
            s_rem = d * ((seg.x[-1] - x) * math.cos(seg.th[-1]) + (seg.y[-1] - y) * math.sin(seg.th[-1]))
        self.err = (e, psi, s_rem)

        # steering: MPC over the next few metres of path, with the current estimate of the gain
        self.gain.update(d, self.delta, pose)
        g = self.gain.g[d]
        mpc = self.mpc
        ahead = s_ref + (np.arange(mpc.N) + 0.5) * mpc.H
        k_ref = np.interp(ahead, seg.s, seg.kappa)           # holds the last curvature past the end
        k_now = g * math.tan(self.delta)
        k_max = g * math.tan(EGO.steer_max)
        # d(kappa)/dt = g (1 + tan^2 delta) d(delta)/dt, and one step takes h / |v|
        dk_max = min(g * (1.0 + math.tan(self.delta) ** 2) * STEER_RATE * mpc.H / max(abs(v), 0.3), 2.0 * k_max)
        K, pred = mpc.solve(d, e, psi, min(max(k_now, -k_max), k_max), k_ref, k_max, dk_max)
        target = math.atan(K[0] / g)
        self.delta += max(-STEER_RATE * dt, min(STEER_RATE * dt, target - self.delta))
        j = np.minimum(np.searchsorted(seg.s, s_ref + (np.arange(mpc.N) + 1.0) * mpc.H), len(seg.x) - 1)
        self.horizon = np.stack([seg.x[j] - pred[:, 0] * np.sin(seg.th[j]),
                                 seg.y[j] + pred[:, 0] * np.cos(seg.th[j])], axis=1)
        self.k_plan, self.k_ref = K, k_ref

        va = v * d
        hold = EGO.torque(A_BRAKE)
        if self.phase == "steer":      # stationary: turn the wheels before moving off
            if (abs(target - self.delta) < 0.02 and t - self.t_phase > 0.3) or t - self.t_phase > 2.5:
                self.phase = "go"
            return self.delta, 0.0, hold
        if self.phase == "go":
            if s_rem <= 0.015 + 0.06 * max(va, 0.0):
                self.phase, self.t_still = "stop", None
            else:
                v_tgt = max(min(seg.v_ref[i], math.sqrt(2.0 * self.A_DEC * max(s_rem, 0.0))), self.V_CREEP)
                v_new = min(v_tgt, self.v_cmd + self.A_ACC * dt)
                feedforward = min(max((v_new - self.v_cmd) / dt, -1.0), 1.0)     # slope of the speed profile
                self.v_cmd = v_new
                err = self.v_cmd - va
                self.integ = min(max(self.integ + self.KI * err * dt, -0.5), 1.0)
                accel = feedforward + self.KP * err + self.integ   # wanted acceleration along the path
                if accel >= 0.0:
                    return self.delta, d * EGO.torque(min(accel, A_DRIVE_MAX)), 0.0
                return self.delta, 0.0, EGO.torque(min(max(-accel - 0.1, 0.0), A_BRAKE))
        # stop: hold the brake until the car has been still for a moment
        if abs(v) < 0.03:
            if self.t_still is None:
                self.t_still = t
            elif t - self.t_still > 0.3:
                self.done, self.phase = True, "idle"
        else:
            self.t_still = None
        return self.delta, 0.0, hold
