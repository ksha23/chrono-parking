"""A stall, and what the map says about it: free or taken, its neighbours, the kerb behind it."""

import math

import numpy as np

from .geometry import wrap
from .vehicle import EGO

KERB_GAP = 0.40      # between the end of the parked car and a kerb that a camera network saw [m]
TICK = 2.5           # how deep a parallel stall is taken to be when only the ends of its ticks were seen [m]


class Slot:
    FREE, OCCUPIED, UNKNOWN = "free", "occupied", "unknown"

    def __init__(self, kind, center, u_in, along, width, depth, corners):
        self.kind = kind            # 'perpendicular' | 'angled' | 'parallel'
        self.center = center        # where the middle of the parked car should be
        self.u_in = u_in            # unit vector from the lane into the stall
        self.along = along          # parallel stalls: unit vector along the kerb
        self.width = width
        self.depth = depth
        self.corners = corners      # entrance A, entrance B, back B, back A
        self.status = Slot.UNKNOWN
        self.neighbors = [False, False]
        self.watched = 0.0          # for how long the less watched of its two lines has been watched [s]
        self.by_row = False         # known from two stubs and the row they stand in, not from a line of its own
        self.region = None          # (mask, slices) of the grid cells between the two lines

    def goal(self, nose_in, travel_dir):
        """Rear-axle pose of the parked car."""
        if self.kind == "parallel":
            t = self.along if self.along @ travel_dir >= 0.0 else -self.along
            th = math.atan2(t[1], t[0])
        else:
            th = math.atan2(self.u_in[1], self.u_in[0]) + (0.0 if nose_in else math.pi)
        return (self.center[0] - EGO.center * math.cos(th),
                self.center[1] - EGO.center * math.sin(th), wrap(th))


def _seen_along(grid, u, nu, s_lo, s_hi, l0, l1):
    """Over what length along nu, between l0 and l1, something stands at a distance s_lo to s_hi
    along u: an obstacle in the map, or a kerb that a camera network saw [m]."""
    box = np.array([u * s + nu * l for s in (s_lo, s_hi) for l in (l0, l1)])
    win = grid.window(box[:, 0].min(), box[:, 1].min(), box[:, 0].max(), box[:, 1].max())
    if win is None:
        return 0.0
    X, Y, sl = win
    S, Lc = X * u[0] + Y * u[1], X * nu[0] + Y * nu[1]
    there = grid.occupied()[sl]
    if getattr(grid, "kerb", None) is not None:
        there = there | (grid.kerb[sl] >= 0.35)
    m = there & (S > s_lo) & (S < s_hi) & (Lc > l0) & (Lc < l1)
    return 0.1 * len(np.unique(np.floor(Lc[m] / 0.1))) if m.any() else 0.0


def _kerb_behind(grid, u, nu, s0, l0, l1, lane=None):
    """Where the kerb at the back of a stall is: the distance along u (into the stall) at which the
    car would meet it, or None if no kerb was seen there. It is read from the map's kerb layer
    between the two lines, from 2.5 to 9.5 m in from the mouth.

    The kerb behind a row runs along the lane, so behind an angled stall it lies at a slant, and
    the answer is for the side of the car that the kerb is nearer to. The slant is that of the
    lane, if the car has driven enough of it to tell (lane: its direction). It is not read from
    what was seen of the kerb: from 12 m away a kerb is smeared along the line of sight, which
    crosses the stall at an angle, and a line fitted to that came out slanted behind a stall the
    kerb is square to. The car then stopped 0.5 m short. Without a lane a line is fitted.

    A kerb seen from far away is smeared over half a metre in range, most of it beyond its face:
    the line is put where the nearest fifth of what was seen begins to count."""
    kerb = getattr(grid, "kerb", None)
    if kerb is None or not kerb.any():
        return None
    box = np.array([u * s + nu * l for s in (s0 + 2.5, s0 + 9.5) for l in (l0, l1)])
    win = grid.window(box[:, 0].min(), box[:, 1].min(), box[:, 0].max(), box[:, 1].max())
    if win is None:
        return None
    X, Y, sl = win
    S, Lc, K = X * u[0] + Y * u[1], X * nu[0] + Y * nu[1], kerb[sl]
    m = (K >= 0.35) & (S > s0 + 2.5) & (S < s0 + 9.5) & (Lc > l0 + 0.2) & (Lc < l1 - 0.2)
    if m.sum() < 8 or np.ptp(Lc[m]) < 1.0:
        return None
    S, Lc, K = S[m], Lc[m] - 0.5 * (l0 + l1), K[m]
    if lane is not None and abs(lane @ nu) > 0.5:
        slant = float((lane @ u) / (lane @ nu))
    else:
        slant = float(np.polyfit(Lc, S, 1, w=np.sqrt(K))[0]) if np.ptp(Lc) >= 1.5 else 0.0
        slant = 0.0 if abs(slant) < 0.25 else slant       # (square to the stall)
    slant = max(-1.2, min(1.2, slant))                    # (up to 50 degrees off)
    level = S - slant * Lc                                # as far in as the kerb is at the middle of the stall
    order = np.argsort(level)
    share = np.cumsum(K[order]) / K.sum()
    return float(level[order][np.searchsorted(share, 0.2)]) - abs(slant) * (EGO.half_width + 0.1)


def _align_with_kerb(slot, grid, s1, l0, l1, lane=None):
    """Parallel stalls: the two short ticks give a poor heading. If the range scan shows the
    kerb behind the stall, park parallel to it and 0.3 m off it, like a driver would.
    lane: the direction of the lane, if the car has driven enough of it to tell."""
    c = slot.corners
    win = grid.window(c[:, 0].min() - 1.5, c[:, 1].min() - 1.5, c[:, 0].max() + 1.5, c[:, 1].max() + 1.5)
    if win is None:
        return
    X, Y, sl = win
    u, nu = slot.u_in, slot.along
    S, Lc = X * u[0] + Y * u[1], X * nu[0] + Y * nu[1]
    m = grid.occupied()[sl] & (S > s1 - 0.6) & (S < s1 + 0.9) & (Lc > l0 + 0.2) & (Lc < l1 - 0.2)
    if m.sum() < 25 or np.ptp(Lc[m]) < 3.5:
        return
    mid = 0.5 * (l0 + l1)
    # the face of the kerb is the lane-side edge of what was hit: a sensor that looks down on the
    # kerb also returns points from its top. Per 0.2 m along the kerb, keep the nearest cells.
    col = np.floor((Lc[m] - l0) / 0.2).astype(int)
    first = np.full(col.max() + 1, np.inf)
    np.minimum.at(first, col, S[m])
    edge = S[m] < first[col] + 0.12
    Le, Se = Lc[m][edge], S[m][edge]
    # A kerb runs along the lane. Something else that was hit in the window, like a corner of the
    # car parked next to the stall, is not on a line along the lane through the middle of the
    # hits, and a line that turns out more than 4 degrees off the lane is not the kerb. (A fit to
    # all the hits let such a corner tilt it by 7 degrees, and the car then set out to correct a
    # heading that was right.)
    b0 = float((lane @ u) / (lane @ nu)) if lane is not None and abs(lane @ nu) > 0.5 else 0.0
    level = Se - b0 * (Le - mid)
    keep = np.abs(level - np.median(level)) < 0.15
    if keep.sum() < 15 or np.ptp(Le[keep]) < 3.5:
        return
    b, a = np.polyfit(Le[keep] - mid, Se[keep], 1)
    if abs(math.atan(b) - math.atan(b0)) > math.radians(4.0):
        return
    along = (nu + b * u) / math.hypot(1.0, b)
    u_k = np.array([along[1], -along[0]])
    if u_k @ u < 0.0:
        u_k = -u_k
    # the fitted line runs through the middle of the kerb's hit cells; its face is ~half a cell nearer
    slot.center = u * (a - 0.05) + nu * mid - u_k * (EGO.half_width + 0.30)
    slot.u_in, slot.along = u_k, along


def _classify(slot, grid, s0, s1, l0, l1, deep=False, stagger=0.0):
    """Free / occupied / unknown from the grid cells inside the stall, plus neighbours.
    deep: the far end of the stall may be out of sight (a sensor rig that looks along the lane).
    stagger: how much further in the stall starts at l1 than at l0 (angled stalls) [m]."""
    c = slot.corners
    win = grid.window(c[:, 0].min() - 3.0, c[:, 1].min() - 3.0, c[:, 0].max() + 3.0, c[:, 1].max() + 3.0)
    if win is None:
        return
    X, Y, sl = win
    u, nu = slot.u_in, np.array([-slot.u_in[1], slot.u_in[0]])
    S = X * u[0] + Y * u[1]
    Lc = X * nu[0] + Y * nu[1]
    occ = grid.occupied()[sl]
    free = grid.clear()[sl]
    lat = 0.35 if slot.kind != "parallel" else 0.6
    back = 0.45
    inside = (S > s0 + 0.1) & (S < s1 - back) & (Lc > l0 + lat) & (Lc < l1 - lat)
    cells = max(int(inside.sum()), 1)
    n_occ = int((occ & inside).sum())
    frac_free = (free & inside).sum() / cells
    # Seen from along the lane, the far end of a stall between two cars lies in the shadow of the
    # nearer car. A parked car would show at the mouth of the stall, so a stall whose mouth is seen
    # to be empty counts as free; what is deeper in comes into view while backing in.
    mouth = inside & (S < s0 + 2.5)
    frac_mouth = (free & mouth).sum() / max(int(mouth.sum()), 1)
    # A camera that looks forward sees into a stall only at a slant, past the corner of the car
    # before it: of an empty stall it sees a wedge, about two thirds of the mouth. A car in the
    # stall would stand in that wedge. So with such a rig a stall is free if most of its mouth is
    # seen empty and the empty ground is seen at least 2 m into it.
    wedge = deep and frac_mouth >= 0.55 and frac_free >= 0.25 and \
        (free & inside).any() and float(S[free & inside].max()) - s0 >= 2.0
    # A stall that is known from its row is taken when the car is level with it (agent.py), and
    # by then the cameras have seen all of it that they are going to. Behind a wide car the
    # wedge is narrower than that. What is left to ask is whether a car could stand in the
    # stall: its front would be in the first 1.2 m, across most of the width.
    front = inside & (S < s0 + 1.2)
    level = deep and slot.by_row and front.any() and (free & front).sum() >= 0.75 * front.sum() and \
        float(S[free & inside].max()) - s0 >= 2.0
    if n_occ >= 4:
        slot.status = Slot.OCCUPIED
    elif n_occ <= 1 and (frac_free >= 0.6 or (frac_mouth >= 0.8 and frac_free >= 0.3) or wedge or level):
        slot.status = Slot.FREE
    # The stalls of an angled row are staggered: the one next to this one starts further in,
    # or further out, and so does the car in it. (A band straight across found the car on one
    # side by its front, and of the car on the other side only what the cameras do not see.)
    enter = s0 + (Lc - (l1 if stagger > 0.0 else l0)) * stagger / max(l1 - l0, 1e-6) if slot.kind == "angled" else s0
    side = (S > enter - 0.3) & (S < enter + s1 - s0 - back)
    reach = 2.4 if slot.kind != "parallel" else 3.0
    slot.neighbors = [bool((occ & side & (Lc < l0 - 0.05) & (Lc > l0 - reach)).sum() >= 4),
                      bool((occ & side & (Lc > l1 + 0.05) & (Lc < l1 + reach)).sum() >= 4)]
    # The cells the planner may take to be free once this stall is chosen. If the far end is out
    # of sight, that has to reach as far as the parked car does, plus the planner's margin: a
    # stall is a place that holds a car. What is really there shows up on the way in.
    end = s1 - 0.2
    if deep:
        end = max(end, slot.center @ u + 0.5 * EGO.length + 0.35)
    slot.region = (S > s0 - 1.0) & (S < end) & (Lc > l0 + 0.1) & (Lc < l1 - 0.1), sl
