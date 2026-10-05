"""Planar geometry: angles, footprints, distances between polygons."""

import math

import numpy as np

from .vehicle import EGO


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rect_poly(x, y, yaw, x0, x1, hw):
    """Corners of a body-frame rectangle x in [x0, x1], y in [-hw, hw] placed at (x, y, yaw)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([(x + px * c - py * s, y + px * s + py * c)
                     for px, py in ((x0, -hw), (x1, -hw), (x1, hw), (x0, hw))])


def ego_poly(pose, margin=0.0):
    return rect_poly(pose[0], pose[1], pose[2], -EGO.rear - margin, EGO.front + margin,
                     EGO.half_width + margin)


def _separated(a, b):
    for poly in (a, b):
        e = np.roll(poly, -1, axis=0) - poly
        n = np.stack([-e[:, 1], e[:, 0]], axis=1)
        pa, pb = a @ n.T, b @ n.T
        if np.any(pa.max(0) < pb.min(0)) or np.any(pb.max(0) < pa.min(0)):
            return True
    return False


def _pts_to_edges(p, poly):
    a = poly
    d = np.roll(poly, -1, axis=0) - poly
    ap = p[:, None, :] - a[None, :, :]
    t = np.clip((ap * d[None]).sum(2) / np.maximum((d * d).sum(1)[None], 1e-12), 0.0, 1.0)
    q = ap - t[..., None] * d[None]
    return np.sqrt((q * q).sum(2)).min()


def poly_distance(a, b):
    """Distance between two convex polygons (0 if they overlap)."""
    if not _separated(a, b):
        return 0.0
    return float(min(_pts_to_edges(a, b), _pts_to_edges(b, a)))


def footprint_hits(poses, pts, margin):
    """poses (N,3), pts (M,2): for every pose, is any point inside the (inflated) ego footprint?"""
    out = np.zeros(len(poses), dtype=bool)
    if len(poses) == 0 or len(pts) == 0:
        return out
    px, py = pts[:, 0].astype(np.float32), pts[:, 1].astype(np.float32)
    for i in range(0, len(poses), 16):       # a few poses at a time: bounded working memory
        q = poses[i:i + 16]
        c, s = np.cos(q[:, 2])[:, None], np.sin(q[:, 2])[:, None]
        dx, dy = px[None, :] - q[:, 0, None], py[None, :] - q[:, 1, None]
        lx = dx * c + dy * s
        ly = dy * c - dx * s
        out[i:i + 16] = ((lx > -EGO.rear - margin) & (lx < EGO.front + margin) &
                         (np.abs(ly) < EGO.half_width + margin)).any(axis=1)
    return out
