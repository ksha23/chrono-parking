"""What the stalls on one side of a lane have in common.

A camera that looks along a lane sees little of a stall between two parked cars: the ends of its
two lines, where they stick out between the cars. Each stall by itself is then two stubs of paint
with no direction and no depth. But the stalls of a row open onto the lane on one line, and their
lines all run the same way. So the row is worked out first, from all the lines on that side, and
a stall is then described by its entrance: the two points where its lines meet the mouth of the
row, the direction of the row, and a depth that is not measured but assumed for the kind of
stall. That is how the published stall detectors describe one (DMPR-PS, Huang et al. 2019, and
Suhr and Jung 2021 among others), and for the same reason: the entrance is what is seen."""

import numpy as np


class Row:
    """The lines on one side of the lane. Distances 'off the path' are measured from the line the
    car drives along, towards this side."""

    def __init__(self, origin, inward, mouth, d, count, longest):
        self.origin, self.inward = origin, inward     # a point of the path, the unit vector from it towards this side
        self.mouth = mouth          # how far off the path the stalls open [m]
        self.d = d                  # the direction of the lines, pointing into the stalls (None: nothing says)
        self.count = count          # lines it was worked out from
        self.longest = longest      # the longest of them [m]: more than a tick, and the row is not parallel stalls

    def off_path(self, q):
        return float((np.asarray(q) - self.origin) @ self.inward)

    def start(self, track):
        """How far off the path the lane-side end of a line was seen."""
        return min(self.off_path(q) for q in track.ends())

    def entrance(self, track, d):
        """Where a line, carried on along d, meets the mouth of the row: the distance along d."""
        return float(track.c @ d + (self.mouth - self.off_path(track.c)) / (d @ self.inward))


def lane_rows(tracks, trail):
    """The rows left and right of the lane the car has come along, as {side: Row} with side +1
    for the left. A side with fewer than two lines that cross the lane's direction has none."""
    trail = np.asarray(trail)
    if len(trail) < 2 or np.hypot(*(trail[-1] - trail[0])) <= 2.0:      # (as find_slots tells the lane)
        return {}
    along = (trail[-1] - trail[0]) / np.hypot(*(trail[-1] - trail[0]))
    left = np.array([-along[1], along[0]])
    seen = {}
    for side in (1, -1):
        inward = side * left
        mine = [t for t in tracks if abs(t.d @ left) > 0.3 and (t.c - trail[0]) @ inward > 0.5]
        if len(mine) < 2:
            continue
        row = Row(trail[0], inward, 0.0, None, len(mine), max(t.length for t in mine))
        # Paint wears off at the lane end of a line and a shadow can hide it, so some lines are
        # seen to start late, by a metre or more. Most are not: the mouth is where the middle
        # one starts.
        row.mouth = float(np.median([row.start(t) for t in mine]))
        seen[side] = row
        # which way the lines run: from those long enough to tell
        long = [t for t in mine if t.length >= 1.5]
        if long:
            d = sum(t.length * (t.d if t.d @ inward > 0.0 else -t.d) for t in long)
            row.d = d / np.hypot(*d)
    # A row of which only stubs were seen takes the direction of the row across the lane,
    # mirrored: the two sides of a lane are laid out alike.
    for side, row in seen.items():
        other = seen.get(-side)
        if row.d is None and other is not None and other.d is not None:
            row.d = other.d - 2.0 * (other.d @ left) * left
    return seen
