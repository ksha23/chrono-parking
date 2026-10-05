"""Reeds-Shepp curves for a unit turning radius."""

import math

import numpy as np


class _M:
    sin, cos, atan2, sqrt, hypot, asin, acos = (math.sin, math.cos, math.atan2, math.sqrt,
                                                math.hypot, math.asin, math.acos)
    fmin, fmax = min, max

    @staticmethod
    def where(c, a, b):
        return a if c else b


class _N:
    sin, cos, atan2, sqrt, hypot, asin, acos = (np.sin, np.cos, np.arctan2, np.sqrt,
                                                np.hypot, np.arcsin, np.arccos)
    fmin, fmax, where = np.minimum, np.maximum, np.where


_HP = 0.5 * math.pi
_Z = 1e-9
_SWAP = str.maketrans("LR", "RL")


def _m2pi(m, x):
    return m.atan2(m.sin(x), m.cos(x))


def _LpSpLp(m, x, y, phi):
    xi, eta = x - m.sin(phi), y - 1.0 + m.cos(phi)
    u, t = m.hypot(xi, eta), m.atan2(eta, xi)
    v = _m2pi(m, phi - t)
    return (t >= -_Z) & (v >= -_Z), t, u, v


def _LpSpRp(m, x, y, phi):
    xi, eta = x + m.sin(phi), y - 1.0 - m.cos(phi)
    u1, t1 = xi * xi + eta * eta, m.atan2(eta, xi)
    u = m.sqrt(m.fmax(u1 - 4.0, 0.0))
    t = _m2pi(m, t1 + m.atan2(2.0, u))
    v = _m2pi(m, t - phi)
    return (u1 >= 4.0) & (t >= -_Z) & (v >= -_Z), t, u, v


def _LpRmL(m, x, y, phi):
    xi, eta = x - m.sin(phi), y - 1.0 + m.cos(phi)
    u1, theta = m.hypot(xi, eta), m.atan2(eta, xi)
    u = -2.0 * m.asin(m.fmin(0.25 * u1, 1.0))
    t = _m2pi(m, theta + 0.5 * u + math.pi)
    v = _m2pi(m, phi - t + u)
    return (u1 <= 4.0) & (t >= -_Z) & (u <= _Z), t, u, v


def _tau_omega(m, u, v, xi, eta, phi):
    delta = _m2pi(m, u - v)
    A = m.sin(u) - m.sin(delta)
    B = m.cos(u) - m.cos(delta) - 1.0
    t1 = m.atan2(eta * A - xi * B, xi * A + eta * B)
    t2 = 2.0 * (m.cos(delta) - m.cos(v) - m.cos(u)) + 3.0
    tau = m.where(t2 < 0.0, _m2pi(m, t1 + math.pi), _m2pi(m, t1))
    return tau, _m2pi(m, tau - u + v - phi)


def _LpRupLumRm(m, x, y, phi):
    xi, eta = x + m.sin(phi), y - 1.0 - m.cos(phi)
    rho = 0.25 * (2.0 + m.hypot(xi, eta))
    u = m.acos(m.fmin(rho, 1.0))
    t, v = _tau_omega(m, u, -u, xi, eta, phi)
    return (rho <= 1.0) & (t >= -_Z) & (v <= _Z), t, u, v


def _LpRumLumRp(m, x, y, phi):
    xi, eta = x + m.sin(phi), y - 1.0 - m.cos(phi)
    rho = (20.0 - xi * xi - eta * eta) / 16.0
    u = -m.acos(m.fmin(m.fmax(rho, 0.0), 1.0))
    t, v = _tau_omega(m, u, u, xi, eta, phi)
    return (rho >= 0.0) & (rho <= 1.0) & (u >= -_HP) & (t >= -_Z) & (v >= -_Z), t, u, v


def _LpRmSmLm(m, x, y, phi):
    xi, eta = x - m.sin(phi), y - 1.0 + m.cos(phi)
    rho, theta = m.hypot(xi, eta), m.atan2(eta, xi)
    r = m.sqrt(m.fmax(rho * rho - 4.0, 0.0))
    u = 2.0 - r
    t = _m2pi(m, theta + m.atan2(r, -2.0))
    v = _m2pi(m, phi - _HP - t)
    return (rho >= 2.0) & (t >= -_Z) & (u <= _Z) & (v <= _Z), t, u, v


def _LpRmSmRm(m, x, y, phi):
    xi, eta = x + m.sin(phi), y - 1.0 - m.cos(phi)
    rho, t = m.hypot(xi, eta), m.atan2(xi, -eta)
    u = 2.0 - rho
    v = _m2pi(m, t + _HP - phi)
    return (rho >= 2.0) & (t >= -_Z) & (u <= _Z) & (v <= _Z), t, u, v


def _LpRmSLmRp(m, x, y, phi):
    xi, eta = x + m.sin(phi), y - 1.0 - m.cos(phi)
    rho = m.hypot(xi, eta)
    u = 4.0 - m.sqrt(m.fmax(rho * rho - 4.0, 0.0))
    t = _m2pi(m, m.atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta))
    v = _m2pi(m, t - phi)
    return (rho >= 2.0) & (u <= _Z) & (t >= -_Z) & (v >= -_Z), t, u, v


def _rs_words(m, x, y, phi):
    """Every Reeds-Shepp word from the origin to (x, y, phi): yields (word, valid, signed lengths)."""
    xb = x * m.cos(phi) + y * m.sin(phi)
    yb = x * m.sin(phi) - y * m.cos(phi)
    for f, r in ((1, 1), (-1, 1), (1, -1), (-1, -1)):     # f: time flip, r: reflection
        X, Y, P, XB, YB = f * x, r * y, f * r * phi, f * xb, r * yb
        w = (lambda s: s) if r > 0 else (lambda s: s.translate(_SWAP))
        ok, t, u, v = _LpSpLp(m, X, Y, P)
        yield w("LSL"), ok, (f * t, f * u, f * v)
        ok, t, u, v = _LpSpRp(m, X, Y, P)
        yield w("LSR"), ok, (f * t, f * u, f * v)
        ok, t, u, v = _LpRmL(m, X, Y, P)
        yield w("LRL"), ok, (f * t, f * u, f * v)
        ok, t, u, v = _LpRmL(m, XB, YB, P)
        yield w("LRL"), ok, (f * v, f * u, f * t)
        ok, t, u, v = _LpRupLumRm(m, X, Y, P)
        yield w("LRLR"), ok, (f * t, f * u, -f * u, f * v)
        ok, t, u, v = _LpRumLumRp(m, X, Y, P)
        yield w("LRLR"), ok, (f * t, f * u, f * u, f * v)
        ok, t, u, v = _LpRmSmLm(m, X, Y, P)
        yield w("LRSL"), ok, (f * t, -f * _HP, f * u, f * v)
        ok, t, u, v = _LpRmSmRm(m, X, Y, P)
        yield w("LRSR"), ok, (f * t, -f * _HP, f * u, f * v)
        ok, t, u, v = _LpRmSmLm(m, XB, YB, P)
        yield w("LSRL"), ok, (f * v, f * u, -f * _HP, f * t)
        ok, t, u, v = _LpRmSmRm(m, XB, YB, P)
        yield w("RSRL"), ok, (f * v, f * u, -f * _HP, f * t)
        ok, t, u, v = _LpRmSLmRp(m, X, Y, P)
        yield w("LRSLR"), ok, (f * t, -f * _HP, f * u, -f * _HP, f * v)


def rs_paths(x, y, phi):
    return [(w, lens) for w, ok, lens in _rs_words(_M, x, y, phi) if ok]


def rs_length_table(xs, ys, phis):
    """Shortest Reeds-Shepp length (unit radius) over a grid of relative poses."""
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    best = np.full(X.shape + (len(phis),), np.inf, dtype=np.float32)
    for k, phi in enumerate(phis):          # one heading at a time keeps the working set small
        P = np.full(X.shape, float(phi))
        for _, ok, lens in _rs_words(_N, X, Y, P):
            total = sum(np.abs(l) for l in lens)
            best[:, :, k] = np.where(ok & (total < best[:, :, k]), total, best[:, :, k])
    return best


def rs_sample(start, word, lens, radius, ds):
    """Integrate a word from a start pose. Rows are (x, y, heading, direction, curvature)."""
    x, y, th = start
    rows = []
    for c, l in zip(word, lens):
        L = abs(l) * radius
        if L < 1e-4:
            continue
        d = 1.0 if l > 0 else -1.0
        n = max(1, int(math.ceil(L / ds)))
        s = np.linspace(L / n, L, n) * d
        if c == "S":
            k = 0.0
            px, py, pth = x + s * math.cos(th), y + s * math.sin(th), np.full(n, th)
        else:
            k = (1.0 if c == "L" else -1.0) / radius
            pth = th + k * s
            px = x + (np.sin(pth) - math.sin(th)) / k
            py = y - (np.cos(pth) - math.cos(th)) / k
        rows.append(np.stack([px, py, pth, np.full(n, d), np.full(n, k)], axis=1))
        x, y, th = px[-1], py[-1], pth[-1]
    return np.concatenate(rows) if rows else np.zeros((0, 5))
