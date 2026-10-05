# Results and limits

## Verification

The same 74 scenarios were run four times: with each of the three sensor sets, and with the
stand-in perception. Each run is a complete search, plan and park, headless. Every figure below is
measured against the **ground truth**, not against the agent's own estimates.

- **Parked**: all four corners of the car inside the stall's lines, the stall truly free, no contact.
- **Lateral, heading, depth**: largest absolute offset from the stall centre and axis over the runs
  in the row. Depth is along the stall.
- **Clearance**: smallest distance between the car's outline and the outline of any parked car or
  kerb at any time during the runs in the row. The outline of a parked car is that of its body,
  without the mirrors (see [simulation-and-viewer.md](simulation-and-viewer.md#scenarios)).
- **Replans**: how often the car stopped in mid-maneuver to plan again, or made a correction
  maneuver at the end.
- **Time**: mean simulated time from start to parked.

**Totals**

| Sensors | Stalls | Parked | Lateral | Heading | Clearance | Gear changes | Runs that replanned |
| --- | --- | --- | --- | --- | --- | --- | --- |
| camera | perpendicular | 26 of 26 | at most 2.0 cm | at most 0.26 deg | at least 0.36 m | 1 to 8 | 1 |
| camera | angled | 24 of 24 | at most 1.3 cm | at most 1.33 deg | at least 0.55 m | 1 to 2 | 1 |
| camera | parallel | 24 of 24 | at most 22.1 cm | at most 0.61 deg | at least 0.14 m | 1 to 5 | 7 |
| camera + lidar | perpendicular | 26 of 26 | at most 3.9 cm | at most 0.53 deg | at least 0.35 m | 1 to 3 | 0 |
| camera + lidar | angled | 24 of 24 | at most 1.4 cm | at most 1.42 deg | at least 0.48 m | 1 to 2 | 0 |
| camera + lidar | parallel | 23 of 24 | at most 3.4 cm | at most 0.88 deg | at least 0.11 m | 2 to 4 | 5 |
| camera + radar | perpendicular | 26 of 26 | at most 3.9 cm | at most 0.45 deg | at least 0.35 m | 1 to 4 | 0 |
| camera + radar | angled | 24 of 24 | at most 1.4 cm | at most 1.40 deg | at least 0.50 m | 0 to 2 | 0 |
| camera + radar | parallel | 24 of 24 | at most 5.3 cm | at most 2.84 deg | at least 0.20 m | 1 to 4 | 4 |
| stand-in (`sim`) | perpendicular | 26 of 26 | at most 2.1 cm | at most 0.59 deg | at least 0.31 m | 1 to 4 | 0 |
| stand-in (`sim`) | angled | 24 of 24 | at most 3.1 cm | at most 1.46 deg | at least 0.33 m | 0 to 2 | 1 |
| stand-in (`sim`) | parallel | 24 of 24 | at most 2.5 cm | at most 0.23 deg | at least 0.23 m | 1 to 4 | 0 |

That is 295 of 296 runs. The one that did not count as parked is with the lidar at double noise:
the car ended inside the lines, 2 cm off centre, but had touched the kerb during a correction
maneuver. See the comments below.

Comments on the numbers that stand out:

- **Double noise** (`--noise 2`, one run per stall type and sensor set). With cameras alone all
  three parked. The parallel one ended 22 cm off the centre of the stall and 39 cm along it, inside
  the lines: the planner had shifted the goal away from an obstacle that the noisy depth put
  there. With the lidar the parallel run touched the kerb. With the radars the parallel run ended
  2.8 degrees off the kerb line. At twice the stereo depth error a 15 cm kerb is at the limit of
  what the depth image can tell from the road, and the kerb is what a parallel stall is aligned
  with. Without the double-noise runs, parallel parking with a sensor rig ends at most 5.4 cm
  off centre and 0.9 degrees off axis, with at least 0.11 m of clearance.
- **Replans.** With cameras, 9 of 74 runs stopped once or twice to plan again, mostly in parallel
  stalls. That is the path monitor reacting to an obstacle that turned out to be nearer than it
  looked from further away. The stand-in perception sees every obstacle from the start and
  replanned once in 74 runs.
- **Gear changes.** The lidar gives the shortest maneuvers, mostly one sweep into a perpendicular
  stall, because the whole lot is on the map when the plan is made. With cameras alone the car
  knows less of the far side of the aisle and the same stall usually takes three gear changes. The
  eight are the perpendicular run at double noise.
- **Depth** offsets of 10 to 18 cm occur in a few angled and hand-placed runs. The far end of a
  stall line is the least well observed part of it, and the car is still well inside the stall.
- **Clearance with the stand-in** is larger than in earlier versions of this document (0.12 m for
  the nose-in perpendicular run then, 0.31 m now). The maneuvers are the same. The parked cars'
  outline used to be the bounding box of the mesh, which is wider than the body by the mirrors.

All four batches used the same script. The sensor batches ran on Chrono main with the Metal RT
backend and the bindings patch, five runs at a time, about 18 minutes per batch. The stand-in
batch ran on the PyChrono 10 conda package.

<details>
<summary>Camera: every row</summary>

74 of 74 parked. Longest run 62 s of simulated time, 9 runs replanned or corrected (10 times in all), peak memory 3.5 GB.

| Stalls | Cars | Variant | Runs parked | Lateral [cm] | Heading [deg] | Depth [cm] | Clearance [m] | Gear changes | Replans | Time [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| perpendicular | both | default | 4 of 4 | 0.6 | 0.23 | 6.0 | 0.54 | 3 | 0 | 40 |
| perpendicular | left | default | 4 of 4 | 0.9 | 0.11 | 5.6 | 0.44 | 3 | 0 | 36 |
| perpendicular | none | default | 4 of 4 | 0.6 | 0.15 | 5.3 | 0.44 | 1 | 0 | 30 |
| perpendicular | random | default | 4 of 4 | 0.5 | 0.22 | 5.7 | 0.44 | 3 | 2 | 39 |
| perpendicular | right | default | 4 of 4 | 0.9 | 0.24 | 5.6 | 0.54 | 1 | 0 | 37 |
| perpendicular | both | noise 2 | 1 of 1 | 0.4 | 0.19 | 1.8 | 0.47 | 8 | 0 | 62 |
| perpendicular | both | park forward | 1 of 1 | 0.9 | 0.26 | 8.3 | 0.42 | 3 | 0 | 61 |
| perpendicular | both | side left | 1 of 1 | 0.4 | 0.03 | 7.0 | 0.42 | 3 | 0 | 43 |
| perpendicular | random | target 17.6,-6.3,90 | 1 of 1 | 2.0 | 0.03 | 13.5 | 0.36 | 2 | 0 | 43 |
| perpendicular | none | target 6.0,0.5,25, no-snap | 1 of 1 | 0.3 | 0.04 | 1.0 | n/a | 1 | 0 | 25 |
| perpendicular | both | tire pac02 | 1 of 1 | 0.4 | 0.10 | 4.9 | 0.53 | 3 | 0 | 42 |
| angled | both | default | 4 of 4 | 0.5 | 0.12 | 0.8 | 0.65 | 2 | 0 | 32 |
| angled | left | default | 4 of 4 | 1.2 | 0.07 | 3.2 | 0.56 | 2 | 0 | 28 |
| angled | none | default | 4 of 4 | 0.9 | 1.33 | 7.0 | 0.64 | 1 to 2 | 1 | 25 |
| angled | random | default | 4 of 4 | 1.3 | 0.96 | 3.2 | 0.58 | 1 to 2 | 0 | 28 |
| angled | right | default | 4 of 4 | 0.8 | 0.14 | 9.1 | 0.59 | 2 | 0 | 35 |
| angled | both | angle 45 | 1 of 1 | 0.6 | 0.11 | 0.2 | 0.70 | 1 | 0 | 28 |
| angled | both | noise 2 | 1 of 1 | 0.0 | 0.01 | 9.6 | 0.64 | 1 | 0 | 39 |
| angled | both | side left | 1 of 1 | 0.5 | 0.09 | 14.1 | 0.55 | 2 | 0 | 32 |
| angled | both | tire pac02 | 1 of 1 | 0.9 | 0.03 | 4.4 | 0.62 | 2 | 0 | 33 |
| parallel | both | default | 4 of 4 | 4.5 | 0.27 | 0.8 | 0.25 | 2 to 4 | 1 | 47 |
| parallel | left | default | 4 of 4 | 4.2 | 0.59 | 2.4 | 0.27 | 1 to 2 | 0 | 36 |
| parallel | none | default | 4 of 4 | 3.6 | 0.21 | 1.6 | 0.20 | 2 | 0 | 33 |
| parallel | random | default | 4 of 4 | 5.4 | 0.61 | 2.6 | 0.26 | 2 | 3 | 42 |
| parallel | right | default | 4 of 4 | 4.3 | 0.08 | 1.0 | 0.27 | 2 | 0 | 46 |
| parallel | both | noise 2 | 1 of 1 | 22.1 | 0.03 | 39.2 | 0.14 | 3 | 1 | 47 |
| parallel | both | side left | 1 of 1 | 3.2 | 0.16 | 0.5 | 0.21 | 5 | 1 | 50 |
| parallel | both | target 18.0,-3.0,0 | 1 of 1 | 3.0 | 0.09 | 0.6 | 0.25 | 2 | 1 | 47 |
| parallel | both | tire pac02 | 1 of 1 | 4.8 | 0.09 | 1.2 | 0.26 | 2 | 0 | 43 |

</details>

<details>
<summary>Camera + lidar: every row</summary>

73 of 74 parked. Longest run 63 s of simulated time, 5 runs replanned or corrected (11 times in all), peak memory 4.1 GB.

| Stalls | Cars | Variant | Runs parked | Lateral [cm] | Heading [deg] | Depth [cm] | Clearance [m] | Gear changes | Replans | Time [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| perpendicular | both | default | 4 of 4 | 1.1 | 0.24 | 5.6 | 0.46 | 1 | 0 | 33 |
| perpendicular | left | default | 4 of 4 | 1.1 | 0.31 | 8.0 | 0.41 | 1 to 2 | 0 | 31 |
| perpendicular | none | default | 4 of 4 | 0.9 | 0.17 | 5.7 | 0.43 | 1 | 0 | 30 |
| perpendicular | random | default | 4 of 4 | 0.8 | 0.21 | 6.4 | 0.36 | 1 to 3 | 0 | 33 |
| perpendicular | right | default | 4 of 4 | 0.8 | 0.23 | 5.6 | 0.54 | 1 | 0 | 37 |
| perpendicular | both | noise 2 | 1 of 1 | 1.9 | 0.45 | 5.4 | 0.54 | 1 | 0 | 32 |
| perpendicular | both | park forward | 1 of 1 | 0.5 | 0.53 | 8.1 | 0.42 | 3 | 0 | 60 |
| perpendicular | both | side left | 1 of 1 | 0.1 | 0.12 | 6.0 | 0.54 | 3 | 0 | 43 |
| perpendicular | random | target 17.6,-6.3,90 | 1 of 1 | 3.9 | 0.45 | 13.8 | 0.35 | 1 | 0 | 37 |
| perpendicular | none | target 6.0,0.5,25, no-snap | 1 of 1 | 0.3 | 0.04 | 1.0 | n/a | 1 | 0 | 25 |
| perpendicular | both | tire pac02 | 1 of 1 | 0.8 | 0.12 | 4.0 | 0.47 | 2 | 0 | 38 |
| angled | both | default | 4 of 4 | 0.6 | 0.12 | 9.1 | 0.48 | 2 | 0 | 32 |
| angled | left | default | 4 of 4 | 1.1 | 0.10 | 3.1 | 0.56 | 2 | 0 | 27 |
| angled | none | default | 4 of 4 | 1.2 | 1.42 | 7.9 | 0.64 | 1 | 0 | 23 |
| angled | random | default | 4 of 4 | 1.4 | 0.99 | 4.4 | 0.56 | 1 to 2 | 0 | 28 |
| angled | right | default | 4 of 4 | 1.3 | 0.99 | 6.6 | 0.61 | 1 | 0 | 31 |
| angled | both | angle 45 | 1 of 1 | 1.3 | 0.20 | 0.1 | 0.63 | 1 | 0 | 27 |
| angled | both | noise 2 | 1 of 1 | 0.4 | 0.07 | 12.0 | 0.56 | 2 | 0 | 31 |
| angled | both | side left | 1 of 1 | 0.2 | 0.11 | 17.7 | 0.60 | 2 | 0 | 32 |
| angled | both | tire pac02 | 1 of 1 | 0.3 | 0.06 | 1.7 | 0.65 | 2 | 0 | 33 |
| parallel | both | default | 4 of 4 | 2.5 | 0.29 | 0.9 | 0.11 | 2 to 4 | 1 | 45 |
| parallel | left | default | 4 of 4 | 1.9 | 0.37 | 0.5 | 0.23 | 2 | 0 | 37 |
| parallel | none | default | 4 of 4 | 3.4 | 0.63 | 1.5 | 0.12 | 2 | 6 | 40 |
| parallel | random | default | 4 of 4 | 2.6 | 0.27 | 0.8 | 0.24 | 2 | 0 | 38 |
| parallel | right | default | 4 of 4 | 1.1 | 0.88 | 1.9 | 0.12 | 2 | 4 | 55 |
| parallel | both | noise 2 | 0 of 1 | n/a | n/a | n/a | n/a | n/a | n/a | n/a |
| parallel | both | side left | 1 of 1 | 1.5 | 0.24 | 1.4 | 0.19 | 2 | 0 | 44 |
| parallel | both | target 18.0,-3.0,0 | 1 of 1 | 2.5 | 0.07 | 1.0 | 0.27 | 2 | 0 | 40 |
| parallel | both | tire pac02 | 1 of 1 | 1.9 | 0.21 | 2.4 | 0.18 | 2 | 0 | 40 |

</details>

<details>
<summary>Camera + radar: every row</summary>

74 of 74 parked. Longest run 66 s of simulated time, 4 runs replanned or corrected (8 times in all), peak memory 4.7 GB.

| Stalls | Cars | Variant | Runs parked | Lateral [cm] | Heading [deg] | Depth [cm] | Clearance [m] | Gear changes | Replans | Time [s] |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| perpendicular | both | default | 4 of 4 | 1.0 | 0.25 | 6.1 | 0.54 | 2 to 3 | 0 | 39 |
| perpendicular | left | default | 4 of 4 | 1.0 | 0.33 | 5.4 | 0.43 | 1 to 3 | 0 | 35 |
| perpendicular | none | default | 4 of 4 | 0.6 | 0.11 | 5.4 | 0.44 | 1 | 0 | 30 |
| perpendicular | random | default | 4 of 4 | 0.7 | 0.22 | 5.5 | 0.44 | 1 to 3 | 0 | 35 |
| perpendicular | right | default | 4 of 4 | 0.8 | 0.24 | 5.6 | 0.54 | 1 | 0 | 37 |
| perpendicular | both | noise 2 | 1 of 1 | 1.0 | 0.02 | 6.2 | 0.56 | 4 | 0 | 64 |
| perpendicular | both | park forward | 1 of 1 | 0.9 | 0.33 | 8.5 | 0.42 | 3 | 0 | 66 |
| perpendicular | both | side left | 1 of 1 | 0.9 | 0.11 | 5.6 | 0.54 | 3 | 0 | 43 |
| perpendicular | random | target 17.6,-6.3,90 | 1 of 1 | 3.9 | 0.45 | 13.7 | 0.35 | 1 | 0 | 37 |
| perpendicular | none | target 6.0,0.5,25, no-snap | 1 of 1 | 0.3 | 0.04 | 1.0 | n/a | 1 | 0 | 25 |
| perpendicular | both | tire pac02 | 1 of 1 | 0.5 | 0.10 | 2.3 | 0.51 | 3 | 0 | 41 |
| angled | both | default | 4 of 4 | 0.5 | 0.10 | 1.0 | 0.67 | 2 | 0 | 32 |
| angled | left | default | 4 of 4 | 1.0 | 0.40 | 3.0 | 0.50 | 2 | 0 | 28 |
| angled | none | default | 4 of 4 | 0.8 | 1.40 | 7.9 | 0.73 | 1 | 0 | 23 |
| angled | random | default | 4 of 4 | 1.2 | 0.23 | 4.1 | 0.56 | 1 to 2 | 0 | 28 |
| angled | right | default | 4 of 4 | 0.8 | 0.56 | 8.4 | 0.62 | 0 to 2 | 0 | 34 |
| angled | both | angle 45 | 1 of 1 | 1.4 | 0.18 | 0.5 | 0.63 | 1 | 0 | 27 |
| angled | both | noise 2 | 1 of 1 | 0.6 | 0.08 | 2.7 | 0.64 | 2 | 0 | 33 |
| angled | both | side left | 1 of 1 | 0.5 | 0.10 | 13.9 | 0.54 | 2 | 0 | 32 |
| angled | both | tire pac02 | 1 of 1 | 0.7 | 0.12 | 10.7 | 0.76 | 2 | 0 | 33 |
| parallel | both | default | 4 of 4 | 4.2 | 0.27 | 0.7 | 0.26 | 2 | 0 | 44 |
| parallel | left | default | 4 of 4 | 4.4 | 0.58 | 2.4 | 0.27 | 2 | 0 | 36 |
| parallel | none | default | 4 of 4 | 3.2 | 0.16 | 1.5 | 0.20 | 2 | 1 | 37 |
| parallel | random | default | 4 of 4 | 5.3 | 0.48 | 1.7 | 0.25 | 1 to 4 | 1 | 41 |
| parallel | right | default | 4 of 4 | 4.0 | 0.22 | 1.7 | 0.26 | 2 | 3 | 49 |
| parallel | both | noise 2 | 1 of 1 | 0.1 | 2.84 | 2.9 | 0.20 | 4 | 3 | 55 |
| parallel | both | side left | 1 of 1 | 3.9 | 0.67 | 0.1 | 0.23 | 4 | 0 | 51 |
| parallel | both | target 18.0,-3.0,0 | 1 of 1 | 3.2 | 0.10 | 0.1 | 0.26 | 4 | 0 | 48 |
| parallel | both | tire pac02 | 1 of 1 | 4.4 | 0.21 | 1.4 | 0.27 | 2 | 0 | 43 |

</details>

<details>
<summary>Stand-in perception: every row</summary>

74 of 74 parked. Longest run 51 s of simulated time, 1 runs replanned or corrected (1 times in all), peak memory 0.2 GB.

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

The default rows are seeds 1 to 4. `noise 2` doubles every perception noise term. The `target`
rows give the car a parking pose instead of letting it choose, and `no-snap` parks at exactly that
pose in open space, so stall-relative clearance does not apply.

## How to reproduce

```
python parking_sim.py --headless --sensors camera --type perpendicular --cars both --seed 3
python parking_sim.py --headless --sensors sim --type parallel --cars both --seed 1
python tests/test_core.py
```

A run prints one `[result]` line and exits with code 0 if the car ended parked. The same options
give the same result every time on the same PyChrono, with or without the window. This was
checked for all four perception sources against the batch output. Between Chrono versions the
numbers differ in the last digits, because the vehicle dynamics do.

`tests/test_core.py` checks the numerical core without running a simulation: every Reeds-Shepp
candidate must end at its goal, the scalar and array implementations must agree, the MPC's QP
solution is compared with the closed form and with a slow reference solver, the pixel rays, the
planar scan and the stripe detector are run on synthetic data with a known answer, and the line
memory and the far-ground layer of the map are exercised.

## What the plots look like

![Signals of a parallel parking run](img/tracking_parallel.png)

The lateral error stays within a few centimetres on the arcs and returns to zero on the docking
run. The steering gains move away from the model value the first time the car turns in each
direction.

## Not tested

- **Dragging with a real mouse.** The drag mode was exercised with a scripted cursor: pixel to
  ground mapping, snapping to a stall, the approach and the park all work. Nobody has yet moved an
  actual mouse over it. It was not rerun with a sensor rig.
- **Other backends and platforms.** The sensor rig ran on macOS on Apple silicon with Metal RT
  only. The bindings patch was built and checked with Vulkan RT in its CPU fallback, but the
  parking simulation was not run on it, and nothing was run with OptiX. The stand-in perception
  was checked on three PyChrono 10 builds (two conda builds and one from source).
- **Other vehicles.** Only the Chrono sedan. The geometry and the sensor mounts are read from the
  model, so another Chrono vehicle with a hull collision shape should work, but none was tried.
  One was ruled out early: the BMW E90 model's front wheel flips over-centre at full lock in
  reverse.
- **The tour and hand-placed targets with a sensor rig** beyond the two target rows above.

## Limits

- **Localization is given.** The car knows its true pose, including roll and pitch. A real system
  would have odometry drift and would need to localise against the map it builds.
- **The sensors are ray traced, the processing is simple.** Lines are found by brightness and
  shape, with no learned detector. The stereo depth is Chrono's exact depth with an error model
  on top, not the output of a stereo matcher. See [sensors.md](sensors.md#limits).
- **No camera looks sideways.** Something low that appears beside the car, or within 2.6 m in
  front of its bumper, is not seen. The lidar does not help there either: it sees nothing lower
  than its own beams within 4.4 m.
- **Kerbs at double noise** are at the limit of the stereo model, see above.
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
  straight-ahead, which one gain per direction cannot capture. The largest final heading errors,
  up to 1.5 degrees, were nose-in angled stalls.
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
