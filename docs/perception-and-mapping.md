# Perception, mapping and stall choice

This covers everything between the true scene and the decision "park in that stall":
`Perception`, `GridMap`, `LineTrack`, `LineMap`, `find_slots` and `ParkingSim._decide`.

## Simulated perception

Perception is simulated from the ground truth scene because the project is about what happens
after perception. The noise model is still meant to be unkind in the ways a real detector is:
range dependent noise, partial observations, occlusion, dropouts and false positives. Every 0.1 s
`Perception.sense(pose)` returns two things. The sensor origin is the middle of the car body.

![One perception frame](img/perception.png)

### Range scan

360 rays, one per degree, 16 m range. Each ray is intersected with every edge of every obstacle
rectangle (parked cars and kerbs). For a ray from origin `o` in direction `d` and an edge from
`a` to `a + e`:

```math
t = \frac{(a - o) \times e}{d \times e}, \qquad
s = \frac{(a - o) \times d}{d \times e}, \qquad
\text{hit if } t > 0,\; 0 \le s \le 1
```

The true range is the smallest `t` over all edges. The measured range adds Gaussian noise with a
3 cm standard deviation, and 2 percent of the rays are dropped. A ray that hits nothing within
16 m reports no return. All rays against all edges is one vectorised NumPy expression.

### Line detections

Each painted line is sampled every 0.25 m. A sample is visible if it is within 12 m and nearer
than the true range of the ray pointing at it, minus 15 cm. Lines hidden behind a parked car are
therefore not detected, and in a full row only the stub of each line near the lane is visible until
the car is level with the gap. Each contiguous run of at least four visible samples becomes one
detection, and then:

| Effect | Model |
| --- | --- |
| sideways endpoint noise | Gaussian, sigma = 0.03 m + 0.012 x range |
| lengthwise endpoint noise | Gaussian, sigma = 0.08 m + 0.03 x range |
| common sideways bias per detection | Gaussian, sigma = 0.02 m |
| dropout | 12 percent of detections vanish |
| fragmentation | 20 percent of long detections are cut to a random part |
| clutter | Poisson(0.25) random segments per frame, 0.8 to 2.5 m long |

`--noise` scales all of these. At 12 m the endpoints are off by about 17 cm sideways and 44 cm
lengthwise, so a single detection is not good enough to park by. The map has to do the work.

## Occupancy grid

`GridMap` covers the scenario with 10 cm cells and keeps two counters per cell.

- `hits`: how many range returns ended in the cell.
- `free`: in how many scans a ray passed through the cell. Rays are sampled every 20 cm up to
  10 cm short of their return.

Two derived maps are used.

```math
\text{occupied} = (\text{hits} \ge 2) \;\wedge\; (\text{hits} > 0.3 \cdot \text{free})
```

The second condition matters. Range noise puts an occasional return into the free cell just in
front of a surface. Without it, obstacles slowly grow outward by one cell and start to block plans
that were made with a small margin. A real surface stops the rays, so its cells have many hits and
few pass-throughs. A cell that rays mostly pass through only caught noise.

```math
\text{blocked} = \text{occupied} \;\vee\; \neg\,\mathrm{dilate}_{2}(\text{free} \ge 1)
```

The planner uses `blocked`: a cell is drivable only if it was seen to be free. The dilation by two
cells closes the gaps between neighbouring rays at long range, where rays are 28 cm apart. The
interior of a parked car and everything behind it are never seen to be free, so they are solid.

![Map and stalls when the car stops to plan](img/mapping.png)

## Line tracks

`LineMap` keeps one `LineTrack` per painted line. A track stores up to 160 detections and is
re-fitted whenever one is added.

**Association.** A detection joins a track if all of these hold, and the track with the smallest
perpendicular distance wins:

- direction: `|sin(angle between them)| < 0.21`, about 12 degrees
- offset: both endpoints within `0.35 m + 0.03 x range` of the track's line
- overlap: the gap between the detection and the track's extent along the line is under 0.8 m

The last rule keeps two collinear but separate lines apart. In a perpendicular lot the lines of the
two rows are exact extensions of each other across a 7 m aisle.

**Fit.** With detection endpoints `p_i` and weights `w_i = 1 / (0.05 + 0.02 r_i)^2` that favour
close detections, the line passes through the weighted centroid `c` and its direction is the
principal axis of the weighted scatter matrix:

```math
c = \frac{\sum w_i p_i}{\sum w_i}, \qquad
S = \sum w_i (p_i - c)(p_i - c)^{\top}, \qquad
\theta = \tfrac{1}{2}\,\mathrm{atan2}\!\left(2 S_{xy},\; S_{xx} - S_{yy}\right)
```

This is total least squares: it minimises perpendicular distances, so it has no preferred axis.

**Extent.** Where the line starts and ends matters as much as where it lies, because the line ends
define the stall entrance. Each detection covers an interval along the line. A weighted coverage
histogram with 20 cm bins is built, and the extent is first taken as the bins with at least a
quarter of the peak coverage. That estimate overshoots, because it follows the longest detections
and lengthwise noise is large. Each end is then replaced by the weighted median of the detection
endpoints that lie within 0.6 m of it, which is unbiased for a line end that was actually seen.

**Housekeeping.** A track is confirmed after 5 detections and 1.2 m of length. Tracks with fewer
than 3 detections are dropped after 2.5 s, which removes clutter. Tracks that turn out to be the
same line (nearly parallel, within 0.25 m, extents touching) are merged.

## From lines to stalls

A stall is the space between two neighbouring, parallel line tracks. `find_slots` looks at every
pair of confirmed tracks whose directions agree within about 8 degrees.

```
                  back
      a_bk +---------------+ b_bk          s: along the lines, into the stall (u_in)
           |               |               l: across the stall
    line a |     stall     | line b
           |               |
      a_in +---------------+ b_in
               entrance
              (lane side)
```

| Kind | Separation of the lines | Line length | Overlap along the lines |
| --- | --- | --- | --- |
| perpendicular or angled | 2.2 to 3.5 m | both at least 3.5 m | at least 3.0 m |
| parallel | 5.0 to 7.8 m | both 1.5 to 3.6 m, no third tick in between | at least 1.2 m |

For a parallel stall the two "lines" are the short ticks painted across the parking strip at its
front and back, so the car ends up perpendicular to them.

**Which end is the entrance.** The end of the line pair that is nearer to where the car has driven
(its trail of past positions) is the lane side.

**Perpendicular or angled.** With `in_a` and `in_b` the positions of the two entrance ends along
the lines and `w` the separation, the skew is `|in_a - in_b| / w`. A 90 degree stall has skew
near 0. A 60 degree stall has skew `cot 60 = 0.58`. Below 0.2 the stall is called perpendicular.

**Where the car should stand.** Sideways it is centred between the lines. Lengthwise it sits in the
part of the stall that both lines cover:

```math
s_c = s_0 + \mathrm{clamp}\!\left(\tfrac{1}{2}(s_1 - s_0),\; 2.65,\; 2.95\right), \qquad
s_0 = \max(a_{in}, b_{in}), \quad s_1 = \min(a_{bk}, b_{bk})
```

The clamp guards against a far end that has not been seen well. The entrance ends are seen often
and from close up, so the estimate is anchored there.

**Parallel stalls and the kerb.** Two 2.5 m ticks give a poor heading: a few centimetres of
endpoint error become degrees. In one test run the tick based heading was off by 2.3 degrees,
which put a corner of the goal pose on the kerb and left no feasible plan. So for parallel stalls
`_align_with_kerb` fits a line to the occupied cells just behind the stall (the kerb, seen by the
range scan over the full 7 m), rejects outliers beyond 15 cm, and places the car parallel to that
line with its side 0.30 m from it. That is also what a driver does.

## Free, occupied or unknown

`_classify` looks at the grid cells inside the stall, shrunk by 0.35 m at the sides (0.6 m for
parallel) so that a neighbour parked close to the line does not count:

| Result | Condition |
| --- | --- |
| occupied | at least 4 occupied cells inside |
| free | at most 1 occupied cell **and** at least 60 percent of the cells seen free in two or more scans |
| unknown | anything else |

"Free" needs positive evidence. A stall the car has not looked into yet is unknown, not free. For
a stall between two parked cars this means the decision usually falls when the car is roughly
level with it, because that is when the interior becomes visible.

The same pass counts occupied cells just outside each line, which tells whether there is a
neighbour on each side.

## Choosing a stall

`ParkingSim._decide` runs every perception tick while the car is searching.

1. Candidates are stalls that are free, whose weaker line has at least 8 detections, that have not
   been rejected by the planner before, and that lie between 8 m behind and 10 m ahead of the car.
2. The candidate with the lowest score leads:

   ```math
   J = |\text{ahead}| + 0.3\,|\text{sideways}| + 1.5 \cdot (\text{number of neighbours})
   ```

   This is "take the first available stall", with a preference for one that has fewer cars next
   to it when two are similarly close.
3. The car commits only when the leader's estimate has settled: it has led for at least 0.55 s,
   and over that time its centre stayed within 12 cm and its axis within 1.2 degrees.

The settling rule exists because an estimate built from a few far detections moves a lot. Early
versions committed as soon as a stall looked free, planned to a pose that then moved by 20 cm or
more as better detections arrived, and had to replan.

**Nose in or back in.** With `--park auto` the car backs into perpendicular stalls, drives nose
first into angled stalls that lean the way it is travelling, and parallel parks by reversing in and
then pulling forward to the middle. Backing into a perpendicular stall needs less aisle width than
driving in, because the steered axle stays out in the aisle.

If the planner finds no way into the chosen stall, the stall is put on a rejected list and the
search continues down the lane.

## What this stage does not do

- It does not estimate the car's own pose. The true pose from Chrono is used to place detections
  in the world.
- Obstacles are static. Nothing moves except the car.
- The stall size thresholds assume ordinary car stalls. They are prior knowledge about parking
  lots, not something learned from the scene.
