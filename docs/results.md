# Results and limits

## Verification

Each run is a complete search, plan and park, headless. Every figure below is measured against the
**ground truth**, not against the agent's own estimates.

- **Parked**: all four corners of the car inside the stall's lines, the stall truly free, no contact.
- **Lateral, heading, depth**: largest absolute offset from the stall centre and axis over the runs
  in the row. Depth is along the stall.
- **Clearance**: smallest distance between the car's outline and the outline of any parked car or
  kerb at any time during the runs in the row. The outline of a parked car is that of its body,
  without the mirrors (see [simulation-and-viewer.md](simulation-and-viewer.md#scenarios)).
- **Replans**: how often the car stopped in mid-maneuver to plan again, or made a correction
  maneuver at the end.
- **Time**: mean simulated time from start to parked.

Two sets of scenarios were run:

| Perception | Scenarios |
| --- | --- |
| cameras (stereo pair, rear, bumper) | 3 stall types x 4 car layouts (both, left, right, none) x 3 seeds, and each stall type twice at double noise: 42 runs |
| cameras + lidar | 3 stall types x 3 car layouts (both, none, random) x 2 seeds: 18 runs |
| stand-in, no sensors | the 74 scenarios of the earlier versions: all layouts, both sides, forward and reverse parking, two tire models, double noise, hand-placed targets |

The seed also picks the sky for the sensors: seed 1 a clear sky with the sun at 41 degrees, seed 2
a low sun, seed 3 an overcast sky. So a third of the camera runs are under each.

The sensor rig is verified on fewer scenarios than the stand-in because a run takes 3 to 4
minutes instead of 10 seconds. Parking forward on request, the left side of the lane, the second
tire model and hand-placed targets were not run with the rig in this batch.

**Totals**

| Perception | Stalls | Parked | Lateral | Heading | Clearance | Gear changes | Runs that replanned |
| --- | --- | --- | --- | --- | --- | --- | --- |
| camera | perpendicular | 14 of 14 | at most 1.7 cm | at most 0.51 deg | at least 0.18 m | 1 to 7 | 2 |
| camera | angled | 14 of 14 | at most 1.8 cm | at most 1.09 deg | at least 0.53 m | 1 to 2 | 1 |
| camera | parallel | 12 of 14 | at most 8.0 cm | at most 2.44 deg | at least 0.20 m | 1 to 2 | 4 |
| camera + lidar | perpendicular | 6 of 6 | at most 1.5 cm | at most 0.08 deg | at least 0.41 m | 1 to 3 | 0 |
| camera + lidar | angled | 6 of 6 | at most 1.3 cm | at most 0.74 deg | at least 0.57 m | 1 to 1 | 0 |
| camera + lidar | parallel | 6 of 6 | at most 5.9 cm | at most 0.81 deg | at least 0.21 m | 1 to 4 | 0 |
| stand-in (`sim`) | perpendicular | 26 of 26 | at most 2.1 cm | at most 0.59 deg | at least 0.31 m | 1 to 4 | 0 |
| stand-in (`sim`) | angled | 24 of 24 | at most 3.1 cm | at most 1.46 deg | at least 0.33 m | 0 to 2 | 1 |
| stand-in (`sim`) | parallel | 24 of 24 | at most 2.5 cm | at most 0.23 deg | at least 0.23 m | 1 to 4 | 0 |
By sky, camera: clear 15 of 15, low 14 of 15, overcast 11 of 12.
By sky, camera + lidar: clear 9 of 9, low 9 of 9.

Comments on the numbers:

- **58 of 60 with the rig.** The two that did not park are both parallel stalls with a car on one
  side, and in both the car drove past without ever being offered the stall. Nothing was hit.
  - *Low sun, car on the left.* The tick line between the free stall and the parked car was seen
    as two pieces, 0.95 m and 0.45 m, with a metre missing between them where the edge of the
    car's shadow runs along it. Neither piece is long enough to be a tick.
  - *Overcast, car on the right.* The parked car hides the part of the tick beside it. What
    showed was 1.4 to 1.5 m long, around the 1.5 m that a tick had to have, so the stall was
    recognised only as the car was leaving it behind.

  Both are rules of the stall inference, not of the sensing: the map had the free space right in
  both runs. The next commit joins pieces of one line across a gap and accepts a shorter tick
  with a rig. With that, both runs park, and the whole set is run again.
- **The sky makes no difference to the rest.** Hard shadows over the stall lines, a low sun and
  an overcast sky give the same result. The line detector compares paint with the road right
  beside it, which does not care how much light there is.
- **Parallel stalls are the least exact**, as with every perception source: up to 8 cm off
  centre and 1.7 degrees off the kerb line, 2.4 degrees at double noise. The car is aligned with a
  kerb that only the stereo pair sees, from 5 to 8 m, and with two tick lines.
- **Gear changes.** Backing into a perpendicular stall between two cars mostly takes three gear
  changes with the rig. When the plan is made, the stall has been seen only as a wedge, and the
  margins are wider than with the stand-in.

The sensor runs used Chrono main with the Metal RT backend and the bindings patch, IGEV++ with its
Middlebury weights and Depth Anything V2 Small, two runs at a time. The stand-in batch ran on the
PyChrono 10 conda package and reproduces the previous batch run for run.

<details>
<summary>Cameras: every row</summary>

40 of 42 parked. Longest run 66 s of simulated time, 7 runs replanned or corrected (9 times in all), peak memory of the simulation 4.3 GB.

| Stalls | Cars | Variant | Runs parked | Lateral [cm] | Heading [deg] | Depth [cm] | Clearance [m] | Gear changes | Replans | Time [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| perpendicular | both | default | 3 of 3 | 0.3 | 0.06 | 6.7 | 0.47 | 3 to 7 | 1 | 49 |
| perpendicular | left | default | 3 of 3 | 1.5 | 0.51 | 6.6 | 0.42 | 1 | 0 | 29 |
| perpendicular | none | default | 3 of 3 | 1.7 | 0.24 | 30.3 | 0.18 | 1 to 3 | 1 | 32 |
| perpendicular | right | default | 3 of 3 | 0.4 | 0.06 | 5.8 | 0.47 | 2 | 0 | 40 |
| perpendicular | both | noise 2 | 2 of 2 | 0.3 | 0.05 | 17.4 | 0.32 | 3 | 0 | 39 |
| angled | both | default | 3 of 3 | 1.5 | 0.96 | 4.4 | 0.58 | 1 | 0 | 29 |
| angled | left | default | 3 of 3 | 1.0 | 0.99 | 4.7 | 0.53 | 1 to 2 | 0 | 27 |
| angled | none | default | 3 of 3 | 1.2 | 1.09 | 2.0 | 0.65 | 1 | 0 | 23 |
| angled | right | default | 3 of 3 | 1.8 | 0.98 | 7.0 | 0.56 | 1 | 0 | 33 |
| angled | both | noise 2 | 2 of 2 | 1.5 | 0.85 | 4.4 | 0.58 | 1 to 2 | 1 | 33 |
| parallel | both | default | 3 of 3 | 5.1 | 1.71 | 2.5 | 0.20 | 2 | 0 | 40 |
| parallel | left | default | 2 of 3 | 6.1 | 0.26 | 1.4 | 0.31 | 1 to 2 | 2 | 41 |
| parallel | none | default | 3 of 3 | 6.7 | 0.15 | 2.3 | 0.28 | 1 to 2 | 3 | 44 |
| parallel | right | default | 2 of 3 | 5.1 | 0.43 | 1.8 | 0.22 | 2 | 1 | 49 |
| parallel | both | noise 2 | 2 of 2 | 8.0 | 2.44 | 3.9 | 0.22 | 2 | 0 | 41 |

</details>

<details>
<summary>Cameras + lidar: every row</summary>

18 of 18 parked. Longest run 46 s of simulated time, 0 runs replanned or corrected (0 times in all), peak memory of the simulation 5.0 GB.

| Stalls | Cars | Variant | Runs parked | Lateral [cm] | Heading [deg] | Depth [cm] | Clearance [m] | Gear changes | Replans | Time [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| perpendicular | both | default | 2 of 2 | 0.5 | 0.01 | 6.3 | 0.53 | 3 | 0 | 39 |
| perpendicular | none | default | 2 of 2 | 1.5 | 0.01 | 8.2 | 0.41 | 1 | 0 | 30 |
| perpendicular | random | default | 2 of 2 | 0.2 | 0.08 | 5.9 | 0.46 | 3 | 0 | 39 |
| angled | both | default | 2 of 2 | 1.3 | 0.74 | 2.6 | 0.57 | 1 | 0 | 29 |
| angled | none | default | 2 of 2 | 0.5 | 0.54 | 5.2 | 0.62 | 1 | 0 | 23 |
| angled | random | default | 2 of 2 | 0.5 | 0.51 | 1.5 | 0.66 | 1 | 0 | 28 |
| parallel | both | default | 2 of 2 | 5.9 | 0.81 | 2.8 | 0.24 | 2 to 4 | 0 | 42 |
| parallel | none | default | 2 of 2 | 2.6 | 0.14 | 3.0 | 0.22 | 1 to 2 | 0 | 40 |
| parallel | random | default | 2 of 2 | 5.4 | 0.24 | 1.5 | 0.21 | 2 to 4 | 0 | 43 |

</details>

<details>
<summary>Stand-in: every row</summary>

74 of 74 parked. Longest run 51 s of simulated time, 1 runs replanned or corrected (1 times in all), peak memory of the simulation 0.3 GB.

| Stalls | Cars | Variant | Runs parked | Lateral [cm] | Heading [deg] | Depth [cm] | Clearance [m] | Gear changes | Replans | Time [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| perpendicular | both | default | 4 of 4 | 1.7 | 0.59 | 3.7 | 0.45 | 1 to 3 | 0 | 36 |
| perpendicular | left | default | 4 of 4 | 1.5 | 0.25 | 3.7 | 0.44 | 1 | 0 | 29 |
| perpendicular | none | default | 4 of 4 | 1.2 | 0.18 | 6.4 | 0.43 | 1 | 0 | 30 |
| perpendicular | random | default | 4 of 4 | 1.8 | 0.42 | 1.3 | 0.48 | 1 | 0 | 31 |
| perpendicular | right | default | 4 of 4 | 2.1 | 0.43 | 1.4 | 0.47 | 1 | 0 | 37 |
| perpendicular | both | noise 2 | 1 of 1 | 0.4 | 0.50 | 3.5 | 0.45 | 1 | 0 | 33 |
| perpendicular | both | park forward | 1 of 1 | 0.1 | 0.49 | 0.7 | 0.31 | 4 | 0 | 47 |
| perpendicular | both | side left | 1 of 1 | 1.6 | 0.20 | 0.8 | 0.46 | 1 | 0 | 37 |
| perpendicular | random | target 17.6,-6.3,90 | 1 of 1 | 0.7 | 0.15 | 1.4 | 0.48 | 1 | 0 | 37 |
| perpendicular | none | target 6.0,0.5,25, no-snap | 1 of 1 | 0.3 | 0.00 | 0.5 | n/a | 1 | 0 | 25 |
| perpendicular | both | tire pac02 | 1 of 1 | 0.3 | 0.03 | 1.1 | 0.47 | 3 | 0 | 41 |
| angled | both | default | 4 of 4 | 1.0 | 1.46 | 0.6 | 0.57 | 1 to 2 | 1 | 32 |
| angled | left | default | 4 of 4 | 0.3 | 1.29 | 1.4 | 0.47 | 1 to 2 | 0 | 27 |
| angled | none | default | 4 of 4 | 1.2 | 0.58 | 4.3 | 0.65 | 0 to 1 | 0 | 19 |
| angled | random | default | 4 of 4 | 0.2 | 1.20 | 3.8 | 0.55 | 0 to 2 | 0 | 27 |
| angled | right | default | 4 of 4 | 0.5 | 0.92 | 3.7 | 0.57 | 1 | 0 | 34 |
| angled | both | angle 45 | 1 of 1 | 0.7 | 0.12 | 0.9 | 0.71 | 1 | 0 | 28 |
| angled | both | noise 2 | 1 of 1 | 3.1 | 0.61 | 9.2 | 0.60 | 1 | 0 | 29 |
| angled | both | side left | 1 of 1 | 0.4 | 0.17 | 3.5 | 0.33 | 1 | 0 | 28 |
| angled | both | tire pac02 | 1 of 1 | 0.8 | 1.26 | 1.2 | 0.62 | 1 | 0 | 28 |
| parallel | both | default | 4 of 4 | 2.4 | 0.21 | 0.9 | 0.27 | 2 | 0 | 43 |
| parallel | left | default | 4 of 4 | 2.5 | 0.16 | 3.9 | 0.26 | 1 to 2 | 0 | 35 |
| parallel | none | default | 4 of 4 | 2.5 | 0.04 | 0.5 | 0.27 | 2 | 0 | 35 |
| parallel | random | default | 4 of 4 | 2.4 | 0.19 | 1.2 | 0.27 | 2 | 0 | 41 |
| parallel | right | default | 4 of 4 | 2.4 | 0.02 | 0.6 | 0.27 | 2 | 0 | 48 |
| parallel | both | noise 2 | 1 of 1 | 2.4 | 0.19 | 4.2 | 0.24 | 2 | 0 | 42 |
| parallel | both | side left | 1 of 1 | 2.2 | 0.23 | 0.0 | 0.31 | 2 | 0 | 43 |
| parallel | both | target 18.0,-3.0,0 | 1 of 1 | 2.4 | 0.01 | 1.4 | 0.23 | 2 | 0 | 40 |
| parallel | both | tire pac02 | 1 of 1 | 2.3 | 0.22 | 4.3 | 0.32 | 4 | 0 | 51 |

</details>

## Not tested

- **Dragging with a real mouse.** The drag mode was exercised with a scripted cursor: pixel to
  ground mapping, snapping to a stall, the approach and the park all work. Nobody has yet moved an
  actual mouse over it. It was not rerun with a sensor rig.
- **With the rig:** parking forward on request, stalls on the left of the lane, the Pacejka tire
  model, hand-placed targets, the tour, and `--stereo rt` beyond single runs.
- **Other backends and platforms.** The sensor rig ran on macOS on Apple silicon with Metal RT
  only. The bindings patch was built and checked with Vulkan RT in its CPU fallback, but the
  parking simulation was not run on it, and nothing was run with OptiX. The scene relies on how
  Metal RT handles light, textures and glass. The stand-in perception was checked on three
  PyChrono 10 builds (two conda builds and one from source).
- **Other vehicles.** Only the Chrono sedan. The geometry and the sensor mounts are read from the
  model, so another Chrono vehicle with a hull collision shape and a mesh with glass materials
  should work, but none was tried. One was ruled out early: the BMW E90 model's front wheel flips
  over-centre at full lock in reverse.

## Limits

- **Localization is given.** The car knows its true pose, including roll and pitch. A real system
  would have odometry drift and would need to localise against the map it builds.
- **The perception is networks plus rules.** Depth comes from two published networks that were
  not trained on this scene. Lines are found by a rule on brightness and shape, with no learned
  detector. See [sensors.md](sensors.md#limits).
- **Nothing looks sideways, and only the front has two cameras.** Something low that appears
  beside the car is not seen. Behind the car, obstacles are placed only within 1.9 m, and a kerb
  is not seen at all.
- **One exposure.** The cameras do not adapt when the car drives from sun into shade.
- **The world is static.** No moving cars or pedestrians. A new obstacle on the path makes the car
  stop and replan, nothing more.
- **The lane is known.** While searching, the car follows a straight lane it is given. It does
  not explore.
- **This car turns wide.** The sedan's kinematic turning radius is 5.95 m at the rear axle. Tight
  maneuvers need an extra back and forth, and the parallel stalls are 7.2 m long to make a single
  reverse sweep possible. A car with a 4.5 m radius would do visibly better with the same code.
- **Planning is conservative about steering.** The planner uses the bicycle model at the declared
  25 degree steering angle. At its 35 degree stop the car turns 15 percent tighter than that
  going forward and 29 percent tighter in reverse, which the controller learns but the planner
  does not use.
- **Forward docking is less precise than reverse.** Going forward on a steady circle the car turns
  a quarter to a third less than a bicycle model at the same wheel angle, and least of all near
  straight-ahead, which one gain per direction cannot capture.
- **Nose-in perpendicular parking is at the edge of what this car can do** in a 7 m aisle. It
  works, with three or four gear changes.
- **Early commitment.** The car can commit to a stall on an estimate that is still poor. The
  planner then refuses it, the car drives on and tries again a few metres later, which costs an
  extra gear change or two.
- **Brake torque, not pressure.** Chrono's brake has no hydraulics, so the brake command is a
  torque. The drive torque acts on the half-shafts with the engine bypassed, which is closer to an
  electric drive unit than to a combustion powertrain.
- **Stall geometry is assumed.** The thresholds that turn line pairs into stalls encode ordinary
  car stalls. Motorcycle bays, double-length stalls or unmarked spaces are not recognised.
