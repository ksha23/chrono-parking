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

The sensor rig is verified on fewer scenarios than the stand-in because a run takes two to four
minutes instead of 10 seconds. Parking forward on request, the left side of the lane, the second
tire model and hand-placed targets were not run with the rig.

**Totals**

| Perception | Stalls | Parked | Lateral | Heading | Clearance | Gear changes | Runs that replanned |
| --- | --- | --- | --- | --- | --- | --- | --- |
| camera | perpendicular | 14 of 14 | at most 0.9 cm | at most 0.17 deg | at least 0.37 m | 1 to 5 | 2 |
| camera | angled | 14 of 14 | at most 1.3 cm | at most 1.20 deg | at least 0.56 m | 1 to 2 | 0 |
| camera | parallel | 14 of 14 | at most 6.0 cm | at most 0.97 deg | at least 0.17 m | 1 to 3 | 0 |
| camera + lidar | perpendicular | 6 of 6 | at most 0.8 cm | at most 0.10 deg | at least 0.45 m | 1 to 3 | 0 |
| camera + lidar | angled | 6 of 6 | at most 1.1 cm | at most 0.61 deg | at least 0.47 m | 0 to 2 | 0 |
| camera + lidar | parallel | 6 of 6 | at most 6.7 cm | at most 0.59 deg | at least 0.23 m | 2 to 2 | 0 |
| stand-in (`sim`) | perpendicular | 26 of 26 | at most 2.7 cm | at most 0.43 deg | at least 0.31 m | 1 to 4 | 0 |
| stand-in (`sim`) | angled | 24 of 24 | at most 1.3 cm | at most 1.50 deg | at least 0.33 m | 0 to 2 | 2 |
| stand-in (`sim`) | parallel | 24 of 24 | at most 2.8 cm | at most 0.47 deg | at least 0.11 m | 1 to 4 | 0 |

WHERE: camera 0 local 42 north, camera + lidar 0 local 18 north
By sky, camera: clear 15 of 15, low 15 of 15, overcast 12 of 12.
By sky, camera + lidar: clear 9 of 9, low 9 of 9.

**How these numbers came about.** The 60 runs with the rig have been run four times, and the
table above is the fourth.

| Batch | Parked | What did not park, and what was changed after it |
| --- | --- | --- |
| first | 58 of 60 | Two parallel stalls with a car on one side were never recognised. A tick line was seen in two pieces under a shadow, and a tick half hidden by the parked car was just under the length a tick had to have. After it: pieces of one line are joined, and a tick may be 1.2 m with a rig |
| second | 59 of 60 | In the empty perpendicular lot under the overcast sky the car parked 40 cm too deep, 9 cm from the kerb. The first piece of one line of the stall is worn away, and the stall was taken to start where that line does. After it: a stall starts on a line along the lane |
| third | 60 of 60 | |
| seeds 4 to 6, never run before | 43 of 45 | The car drove past two free perpendicular stalls. The map was right in both. A line counted after 5 sightings and a stall after 8, numbers from when a camera fed the map ten times per second, and the stereo network ran 2.5 times per second. After it: nothing counts sightings or frames any more. The maps add up the time each answer stands for, and the networks run 5 times per second ([sensors.md](sensors.md#how-often-the-networks-run)). The check of that on recorded drives turned up one more rule, which was changed too: in a 60 degree lot a half-seen line could be matched to the wrong end of its neighbour |
| fourth: the table above | 60 of 60 | |
| seeds 4 to 6 again | 44 of 45 | see below |

The first three changes were rules of the stall inference meeting the worn paint and the shadows
of the new scene. The fourth was a wrong unit: frames where time was meant.

**That makes seeds 1 to 6 runs the method was fixed on, not a test of it.** Seeds 4 to 6 with the
final code:

| Perception | Stalls | Parked | Lateral | Heading | Clearance | Gear changes | Runs that replanned |
| --- | --- | --- | --- | --- | --- | --- | --- |
| camera | perpendicular | 11 of 12 | at most 1.0 cm | at most 0.25 deg | at least 0.41 m | 1 to 4 | 0 |
| camera | angled | 12 of 12 | at most 1.2 cm | at most 1.12 deg | at least 0.54 m | 1 to 2 | 0 |
| camera | parallel | 12 of 12 | at most 2.4 cm | at most 0.83 deg | at least 0.20 m | 1 to 4 | 2 |
| camera + lidar | perpendicular | 3 of 3 | at most 1.5 cm | at most 0.23 deg | at least 0.47 m | 1 to 3 | 0 |
| camera + lidar | angled | 3 of 3 | at most 0.6 cm | at most 0.21 deg | at least 0.66 m | 0 to 2 | 0 |
| camera + lidar | parallel | 3 of 3 | at most 1.8 cm | at most 0.48 deg | at least 0.19 m | 2 to 2 | 1 |

By sky, camera: clear 12 of 12, low 12 of 12, overcast 11 of 12.
By sky, camera + lidar: clear 3 of 3, low 3 of 3, overcast 3 of 3.

One run of the 45 did not park (perpendicular, a car on the left, seed 6, low sun). The car
stopped in the middle of its manoeuvre, found no way on and gave up, 1.1 m from anything. It is
the only failure so far in which the map was wrong:

- The car had paused for 1.5 s between going forward and reversing, with a parked bus 3 m ahead.
- Along the edge of the car's own bonnet in the image, the stereo network put a few dozen pixels
  somewhere between the bonnet, 1 m away, and the bus. Those points lie in the air 20 to 30 cm
  off the car's own wing.
- While the car drives, such points fall into a different map cell in every frame and never
  count. While it stood, they fell into the same cell four times, and the cell became an
  obstacle that the car could not move away from with the margin the planner keeps.

It was left as it is. A fix was tried: leaving out a band of pixels around the car's own body in
each image. That run then parks, and of the 60 runs none had to stop and plan again. But the band
also hides the nearest strip of road from the pair, and a nose-in run into an angled stall, which
had parked before, then ended 69 cm too deep. So the band is not in this version. A run is
repeatable, so this one can be made again exactly:
`--sensors camera --type perpendicular --cars left --seed 6`.

For a test, 45 runs with seeds that had never been run (7, 8 and 9: other cars, other positions,
other worn paint, each of the three skies) were made once, with the final code:

| Perception | Stalls | Parked | Lateral | Heading | Clearance | Gear changes | Runs that replanned |
| --- | --- | --- | --- | --- | --- | --- | --- |
| camera | perpendicular | 12 of 12 | at most 1.2 cm | at most 0.16 deg | at least 0.47 m | 1 to 8 | 3 |
| camera | angled | 12 of 12 | at most 0.9 cm | at most 1.03 deg | at least 0.54 m | 1 to 2 | 0 |
| camera | parallel | 12 of 12 | at most 5.7 cm | at most 0.43 deg | at least 0.25 m | 1 to 4 | 3 |
| camera + lidar | perpendicular | 3 of 3 | at most 1.2 cm | at most 0.25 deg | at least 0.49 m | 1 to 2 | 0 |
| camera + lidar | angled | 3 of 3 | at most 1.5 cm | at most 0.94 deg | at least 0.61 m | 0 to 2 | 0 |
| camera + lidar | parallel | 3 of 3 | at most 4.4 cm | at most 0.68 deg | at least 0.15 m | 2 to 2 | 0 |

By sky, camera: clear 12 of 12, low 12 of 12, overcast 12 of 12.
By sky, camera + lidar: clear 3 of 3, low 3 of 3, overcast 3 of 3.

That is 45 of 45.

Six of the 36 camera runs stopped once to plan again. One of them, a perpendicular stall
between two cars, then needed eight gear changes and 64 s. The smallest clearance was 0.15 m, in a
parallel stall with the lidar that was planned with a margin of 0.20 m.

Other comments:

- **Twice as often is better.** The third batch had the networks at 2.5 answers per second, the
  fourth has them at 5. On the same 60 scenarios, 2 runs had to stop and plan again instead of 8,
  the worst run took 5 gear changes instead of 7, and the largest heading error went from 2.5 to
  1.2 degrees.
- **The sky makes no difference.** Hard shadows over the stall lines, a low sun and an overcast
  sky give the same result. The line detector compares paint with the road right beside it, which
  does not care how much light there is.
- **Parallel stalls are the least exact**, as with every perception source: up to 7 cm off
  centre and 1 degree off the kerb line. The car is aligned with a kerb that only the stereo pair
  sees, from 5 to 8 m, and with two tick lines.
- **Gear changes.** Backing into a perpendicular stall between two cars mostly takes three gear
  changes with the rig, and up to eight. When the plan is made, the stall has been seen only as a
  wedge, and the margins are wider than with the stand-in.
- **A run can be repeated.** The same scenario gives the same messages at the same times and the
  same result line. That took one setting: the stereo network on a CUDA GPU does not give the
  same disparity twice unless cuDNN is told to ([sensors.md](sensors.md#what-it-needs)).

The sensor runs used Chrono main with the Metal RT backend and the bindings patch, IGEV++ with its
Middlebury weights and Depth Anything V2 Small. The simulation and the cameras ran on an Apple
M4 Pro, four runs at a time, and the networks on an RTX 5070 Ti in another machine
([sensors.md](sensors.md#what-it-needs)), both networks five times per second. A run with the
rig can be repeated: the same scenario gives the same result line to the last digit. The
stand-in batch ran on the PyChrono 10 conda package. It parks the same 74 as the earlier
versions, with the same averages. The single runs moved a little when the map went from counting
scans to adding up time: an obstacle now has to be seen for 0.5 s, which is five scans of the
stand-in, where it used to be two. The smallest clearance of the set is now 0.11 m, in a parallel
stall between two cars that was planned with a margin of 0.12 m. It used to be 0.23 m.

<details>
<summary>Cameras: every row</summary>

42 of 42 parked. Longest run 53 s of simulated time, 2 runs replanned or corrected (2 times in all), peak memory of the simulation 4.3 GB.

| Stalls | Cars | Variant | Runs parked | Lateral [cm] | Heading [deg] | Depth [cm] | Clearance [m] | Gear changes | Replans | Time [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| perpendicular | both | default | 3 of 3 | 0.5 | 0.10 | 6.4 | 0.48 | 3 | 1 | 41 |
| perpendicular | left | default | 3 of 3 | 0.9 | 0.17 | 2.3 | 0.41 | 1 to 3 | 0 | 34 |
| perpendicular | none | default | 3 of 3 | 0.5 | 0.04 | 1.5 | 0.48 | 1 | 0 | 30 |
| perpendicular | right | default | 3 of 3 | 0.4 | 0.07 | 5.6 | 0.48 | 1 to 5 | 0 | 41 |
| perpendicular | both | noise 2 | 2 of 2 | 0.5 | 0.09 | 6.6 | 0.37 | 3 | 1 | 41 |
| angled | both | default | 3 of 3 | 0.9 | 0.28 | 2.6 | 0.63 | 2 | 0 | 32 |
| angled | left | default | 3 of 3 | 0.9 | 0.29 | 2.1 | 0.56 | 2 | 0 | 28 |
| angled | none | default | 3 of 3 | 0.9 | 1.20 | 2.4 | 0.66 | 1 | 0 | 22 |
| angled | right | default | 3 of 3 | 1.3 | 1.03 | 7.8 | 0.61 | 1 | 0 | 33 |
| angled | both | noise 2 | 2 of 2 | 0.6 | 0.14 | 3.0 | 0.65 | 2 | 0 | 32 |
| parallel | both | default | 3 of 3 | 5.3 | 0.46 | 1.7 | 0.19 | 2 to 3 | 0 | 42 |
| parallel | left | default | 3 of 3 | 3.3 | 0.05 | 2.5 | 0.28 | 1 to 2 | 0 | 36 |
| parallel | none | default | 3 of 3 | 3.2 | 0.04 | 3.1 | 0.27 | 1 to 2 | 0 | 36 |
| parallel | right | default | 3 of 3 | 4.4 | 0.73 | 3.0 | 0.24 | 2 to 3 | 0 | 51 |
| parallel | both | noise 2 | 2 of 2 | 6.0 | 0.97 | 1.0 | 0.17 | 2 | 0 | 40 |

</details>

<details>
<summary>Cameras + lidar: every row</summary>

18 of 18 parked. Longest run 41 s of simulated time, 0 runs replanned or corrected (0 times in all), peak memory of the simulation 5.0 GB.

| Stalls | Cars | Variant | Runs parked | Lateral [cm] | Heading [deg] | Depth [cm] | Clearance [m] | Gear changes | Replans | Time [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| perpendicular | both | default | 2 of 2 | 0.8 | 0.09 | 5.8 | 0.45 | 1 to 3 | 0 | 37 |
| perpendicular | none | default | 2 of 2 | 0.5 | 0.01 | 1.5 | 0.48 | 1 | 0 | 30 |
| perpendicular | random | default | 2 of 2 | 0.4 | 0.10 | 2.9 | 0.46 | 1 to 3 | 0 | 34 |
| angled | both | default | 2 of 2 | 1.1 | 0.13 | 0.5 | 0.47 | 2 | 0 | 32 |
| angled | none | default | 2 of 2 | 0.4 | 0.28 | 2.0 | 0.65 | 0 to 1 | 0 | 21 |
| angled | random | default | 2 of 2 | 0.6 | 0.61 | 5.6 | 0.62 | 0 to 2 | 0 | 24 |
| parallel | both | default | 2 of 2 | 5.0 | 0.59 | 1.6 | 0.24 | 2 | 0 | 40 |
| parallel | none | default | 2 of 2 | 4.6 | 0.48 | 2.9 | 0.29 | 2 | 0 | 36 |
| parallel | random | default | 2 of 2 | 6.7 | 0.24 | 1.1 | 0.23 | 2 | 0 | 39 |

</details>

<details>
<summary>Stand-in: every row</summary>

74 of 74 parked. Longest run 50 s of simulated time, 2 runs replanned or corrected (2 times in all), peak memory of the simulation 0.2 GB.

| Stalls | Cars | Variant | Runs parked | Lateral [cm] | Heading [deg] | Depth [cm] | Clearance [m] | Gear changes | Replans | Time [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| perpendicular | both | default | 4 of 4 | 1.8 | 0.37 | 3.5 | 0.42 | 1 to 2 | 0 | 35 |
| perpendicular | left | default | 4 of 4 | 1.4 | 0.28 | 3.7 | 0.44 | 1 | 0 | 29 |
| perpendicular | none | default | 4 of 4 | 1.2 | 0.17 | 3.3 | 0.46 | 1 | 0 | 30 |
| perpendicular | random | default | 4 of 4 | 2.2 | 0.40 | 3.6 | 0.45 | 1 | 0 | 31 |
| perpendicular | right | default | 4 of 4 | 2.7 | 0.43 | 2.2 | 0.47 | 1 | 0 | 37 |
| perpendicular | both | noise 2 | 1 of 1 | 1.5 | 0.14 | 10.5 | 0.60 | 1 | 0 | 33 |
| perpendicular | both | park forward | 1 of 1 | 0.4 | 0.22 | 0.9 | 0.31 | 4 | 0 | 47 |
| perpendicular | both | side left | 1 of 1 | 1.2 | 0.22 | 0.9 | 0.42 | 1 | 0 | 36 |
| perpendicular | random | target 17.6,-6.3,90 | 1 of 1 | 0.7 | 0.16 | 3.9 | 0.45 | 1 | 0 | 37 |
| perpendicular | none | target 6.0,0.5,25, no-snap | 1 of 1 | 0.3 | 0.00 | 0.5 | n/a | 1 | 0 | 25 |
| perpendicular | both | tire pac02 | 1 of 1 | 0.3 | 0.16 | 3.0 | 0.45 | 3 | 0 | 42 |
| angled | both | default | 4 of 4 | 1.0 | 1.50 | 1.0 | 0.57 | 1 to 2 | 1 | 32 |
| angled | left | default | 4 of 4 | 1.0 | 1.31 | 2.8 | 0.47 | 1 to 2 | 0 | 27 |
| angled | none | default | 4 of 4 | 1.1 | 0.63 | 4.0 | 0.65 | 0 to 1 | 0 | 19 |
| angled | random | default | 4 of 4 | 0.3 | 1.21 | 4.2 | 0.56 | 0 to 2 | 0 | 27 |
| angled | right | default | 4 of 4 | 1.0 | 0.81 | 3.6 | 0.57 | 1 | 0 | 34 |
| angled | both | angle 45 | 1 of 1 | 0.5 | 0.12 | 1.6 | 0.70 | 1 | 0 | 28 |
| angled | both | noise 2 | 1 of 1 | 1.3 | 0.02 | 2.3 | 0.62 | 2 | 1 | 36 |
| angled | both | side left | 1 of 1 | 0.1 | 0.01 | 0.7 | 0.33 | 1 | 0 | 28 |
| angled | both | tire pac02 | 1 of 1 | 0.7 | 1.25 | 6.1 | 0.62 | 1 | 0 | 28 |
| parallel | both | default | 4 of 4 | 2.4 | 0.26 | 1.0 | 0.11 | 2 | 0 | 43 |
| parallel | left | default | 4 of 4 | 2.5 | 0.24 | 2.0 | 0.26 | 1 to 2 | 0 | 35 |
| parallel | none | default | 4 of 4 | 2.5 | 0.04 | 0.7 | 0.27 | 2 | 0 | 36 |
| parallel | random | default | 4 of 4 | 2.3 | 0.23 | 1.2 | 0.16 | 2 | 0 | 40 |
| parallel | right | default | 4 of 4 | 2.5 | 0.04 | 0.8 | 0.27 | 2 | 0 | 49 |
| parallel | both | noise 2 | 1 of 1 | 2.8 | 0.47 | 1.3 | 0.25 | 4 | 0 | 50 |
| parallel | both | side left | 1 of 1 | 2.3 | 0.10 | 0.2 | 0.27 | 2 | 0 | 43 |
| parallel | both | target 18.0,-3.0,0 | 1 of 1 | 2.4 | 0.03 | 1.7 | 0.22 | 2 | 0 | 40 |
| parallel | both | tire pac02 | 1 of 1 | 2.4 | 0.04 | 4.1 | 0.23 | 2 | 0 | 45 |

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
