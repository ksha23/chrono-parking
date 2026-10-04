# Results and limits

## Verification batch

74 headless runs, each a complete search, plan and park. Every figure below is measured against
the **ground-truth** stall, not against the agent's own estimate of it.

- **Parked**: all four corners of the car inside the stall's lines, the stall truly free, no contact.
- **Lateral, heading, depth**: largest absolute offset from the stall centre and axis over the runs
  in the row. Depth is along the stall.
- **Clearance**: smallest distance between the car's outline and any parked car or kerb at any
  time during the runs in the row.
- **Time**: mean simulated time from start to parked.

| Stalls | Cars | Variant | Runs parked | Lateral [cm] | Heading [deg] | Depth [cm] | Clearance [m] | Gear changes | Time [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| perpendicular | both | default | 4 of 4 | 2.4 | 0.48 | 8.8 | 0.38 | 1 | 32 |
| perpendicular | left | default | 4 of 4 | 1.1 | 0.11 | 10.3 | 0.38 | 1 to 3 | 33 |
| perpendicular | none | default | 4 of 4 | 0.7 | 0.30 | 8.2 | 0.49 | 1 | 31 |
| perpendicular | random | default | 4 of 4 | 2.3 | 0.45 | 7.0 | 0.37 | 1 | 32 |
| perpendicular | right | default | 4 of 4 | 3.8 | 0.74 | 5.6 | 0.51 | 1 | 36 |
| perpendicular | both | noise 2 | 1 of 1 | 0.3 | 0.12 | 9.7 | 0.58 | 3 | 40 |
| perpendicular | both | park forward | 1 of 1 | 0.8 | 0.66 | 5.3 | 0.23 | 4 | 46 |
| perpendicular | both | side left | 1 of 1 | 0.6 | 0.25 | 1.9 | 0.50 | 3 | 43 |
| perpendicular | random | target 17.6,-6.3,90 | 1 of 1 | 0.6 | 0.07 | 0.6 | 0.49 | 1 | 36 |
| perpendicular | none | target 6.0,0.5,25, no-snap | 1 of 1 | 0.2 | 0.08 | 3.3 | n/a | 1 | 24 |
| perpendicular | both | tire pac02 | 1 of 1 | 0.3 | 0.24 | 7.3 | 0.55 | 3 | 40 |
| angled | both | default | 4 of 4 | 1.0 | 0.23 | 7.9 | 0.43 | 1 to 2 | 32 |
| angled | left | default | 4 of 4 | 0.8 | 0.41 | 2.4 | 0.41 | 2 | 28 |
| angled | none | default | 4 of 4 | 0.6 | 0.80 | 4.3 | 0.64 | 0 to 1 | 22 |
| angled | random | default | 4 of 4 | 0.8 | 0.45 | 3.2 | 0.49 | 1 to 2 | 29 |
| angled | right | default | 4 of 4 | 2.0 | 2.01 | 1.9 | 0.38 | 1 | 33 |
| angled | both | angle 45 | 1 of 1 | 0.1 | 0.02 | 0.6 | 0.58 | 1 | 30 |
| angled | both | noise 2 | 1 of 1 | 0.3 | 0.49 | 1.2 | 0.45 | 2 | 33 |
| angled | both | side left | 1 of 1 | 0.3 | 0.23 | 0.5 | 0.48 | 2 | 33 |
| angled | both | tire pac02 | 1 of 1 | 0.8 | 0.21 | 2.8 | 0.61 | 2 | 33 |
| parallel | both | default | 4 of 4 | 2.8 | 0.19 | 4.1 | 0.25 | 2 | 42 |
| parallel | left | default | 4 of 4 | 2.6 | 0.26 | 4.0 | 0.26 | 2 | 36 |
| parallel | none | default | 4 of 4 | 2.5 | 0.13 | 4.7 | 0.26 | 2 | 35 |
| parallel | random | default | 4 of 4 | 2.5 | 0.28 | 3.6 | 0.27 | 2 | 41 |
| parallel | right | default | 4 of 4 | 2.6 | 0.10 | 3.3 | 0.26 | 2 | 48 |
| parallel | both | noise 2 | 1 of 1 | 2.2 | 0.05 | 4.0 | 0.21 | 2 | 43 |
| parallel | both | side left | 1 of 1 | 2.4 | 0.07 | 4.4 | 0.26 | 2 | 41 |
| parallel | both | target 18.0,-3.0,0 | 1 of 1 | 2.2 | 0.30 | 4.8 | 0.29 | 2 | 44 |
| parallel | both | tire pac02 | 1 of 1 | 2.5 | 0.03 | 3.5 | 0.26 | 2 | 44 |

The default rows are seeds 1 to 4. `noise 2` doubles every perception noise term. The `target`
rows give the car a parking pose instead of letting it choose, and `no-snap` parks at exactly that
pose in open space, so stall-relative clearance does not apply.

**Totals**

| | Parked | Lateral | Heading | Clearance | Gear changes |
| --- | --- | --- | --- | --- | --- |
| perpendicular | 25 of 25 | at most 3.8 cm | at most 0.74 deg | at least 0.23 m | 1 to 4 |
| angled | 24 of 24 | at most 2.0 cm | at most 2.01 deg | at least 0.38 m | 0 to 2 |
| parallel | 24 of 24 | at most 2.8 cm | at most 0.30 deg | at least 0.21 m | 2 |

No run needed a replan or a correction maneuver. The longest took 49 s of simulated time.

Earlier versions of the pipeline went through larger batches (116 runs including left-hand stalls
for every type, forward parking with one neighbour, and zero noise) with the same outcome. Those
ran before the controller was replaced by the MPC, so they are not counted here.

## How to reproduce

```
python parking_sim.py --headless --type perpendicular --cars both --seed 3
python tests/test_core.py
```

A run prints one `[result]` line and exits with code 0 if the car ended parked. The same options
give the same result every time, with or without the window.

`tests/test_core.py` checks the numerical core without running a simulation: every Reeds-Shepp
candidate must end at its goal, the scalar and array implementations must agree, and the MPC's QP
solution is compared with the closed form and with a slow reference solver.

## What the plots look like

![Signals of a parallel parking run](img/tracking_parallel.png)

The lateral error stays within a few centimetres on the arcs and returns to zero on the docking
run. The steering gains move away from the model value the first time the car turns in each
direction.

## Not tested

- **Dragging with a real mouse.** The drag mode was exercised with a scripted cursor: pixel to
  ground mapping, snapping to a stall, the approach and the park all work. Nobody has yet moved an
  actual mouse over it.
- **Other platforms.** Everything ran on macOS on Apple silicon. Headless runs were checked on
  three PyChrono 10 builds (two conda builds and one from source). The window was only run on one
  of them. Drag mode cannot work off macOS as written.
- **Other vehicles.** Only the Chrono sedan. The geometry is read from the model, so another
  Chrono vehicle with a hull collision shape should work, but none was tried. One was ruled out
  early: the BMW E90 model's front wheel flips over-centre at full lock in reverse.

## Limits

- **Localization is given.** The car knows its true pose. A real system would have odometry drift
  and would need to localise against the map it builds.
- **Perception is simulated.** The noise is designed to be awkward, but it is a model. There is no
  camera image and no detector.
- **The world is static.** No moving cars or pedestrians. A new obstacle on the path makes the car
  stop and replan, nothing more.
- **The lane is known.** While searching, the car follows a straight lane it is given. It does
  not explore.
- **This car turns wide.** The sedan's kinematic turning radius is 5.95 m at the rear axle. Tight
  maneuvers need an extra back and forth, and the parallel stalls are 7.2 m long to make a single
  reverse sweep possible. A car with a 4.5 m radius would do visibly better with the same code.
- **Planning is conservative about steering.** The planner uses the model's kinematic limit. The
  car can actually turn 15 percent tighter forward and 29 percent tighter in reverse, which the
  controller learns but the planner does not use.
- **Forward docking is less precise than reverse.** The forward steering response has slack near
  straight-ahead that one gain cannot capture. The largest final heading error, 2.0 degrees, was a
  nose-in angled stall.
- **Early commitment.** The car can commit to a stall on an estimate that is still poor. The
  planner then refuses it, the car drives on and tries again a few metres later, which costs an
  extra gear change or two. That happened in one of the 74 runs.
- **Stall geometry is assumed.** The thresholds that turn line pairs into stalls encode ordinary
  car stalls. Motorcycle bays, double-length stalls or unmarked spaces are not recognised.
