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
- **Committed to a stall, and completed**: of the runs in which the car chose a stall and set
  out for it, those that parked. The others drove past, which is a miss of another kind.
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

**Totals, as the car is now.** The car is told only what a real car would know
([sensors.md](sensors.md#limits)): a pose with the error of a satellite receiver with an inertial
unit (10 cm and 0.3 degrees, wandering slowly), its speed from a wheel encoder, its pitch, roll
and height from the road it sees, and that it starts in a lane with stalls somewhere along it.
From the moment it has chosen a stall it carries its position on by its wheels. The paint is
worn, the road is uneven by 1.5 cm, and the simulation step is 1 ms. A network that labels
each image points out faint paint and kerbs. A stall can be taken from the two ends of its
lines and the row it stands in
([sensors.md](sensors.md#between-two-cars-a-stall-is-two-stubs)), or from one of its lines
where the other was not found ([sensors.md](sensors.md#one-line-found-the-other-not)). Means
and largest offsets are over the runs that parked. All of these runs were rendered with OptiX
on an RTX 5070 Ti.

**Two numbers.** A run can miss in two ways. The car can drive past a free stall because it
does not recognise it. Every stall detector does that some of the time, and it costs the stall.
Or it can choose a stall and then touch something, stop half-way or end over the line. That is
a failure of the car. So each set is given as how many runs parked, and of the runs in which
the car committed to a stall, how many it completed: all four corners inside the lines, nothing
touched.

**Seeds 28 to 30 had never been run.** Each scenario was run once with the code as it is and
once with the version before, both on the same RTX 5070 Ti.

| Perception, unseen seeds 28 to 30 | Parked | Committed to a stall, and completed | Lateral: mean, worst | Depth: mean, worst | Heading: mean, worst | Smallest clearance | Gear changes: mean, most | Time |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| cameras | 34 of 36 | 34 of 34 | 3.7, at most 16.4 cm | 12.0, at most 31.2 cm | 0.4, at most 1.1 deg | 0.20 m | 1.6, 3 | 38 s |
| cameras + lidar | 8 of 9 | 8 of 8 | 3.7, at most 14.5 cm | 10.0, at most 24.3 cm | 0.4, at most 0.9 deg | 0.18 m | 1.9, 3 | 36 s |
| cameras, the version before | 33 of 36 | 33 of 33 | 5.4, at most 13.5 cm | 9.3, at most 31.6 cm | 0.9, at most 2.9 deg | 0.14 m | 1.8, 3 | 36 s |
| cameras + lidar, the version before | 8 of 9 | 8 of 8 | 6.2, at most 10.7 cm | 9.6, at most 29.4 cm | 0.7, at most 1.8 deg | 0.23 m | 1.6, 3 | 34 s |

That is 42 of 45 against 41 of 45. In every run in which the car committed to a stall it
completed it, with both versions: 42 of 42 and 41 of 41, and nothing was touched. One scenario
that drove past before parks now, the last stall of a row. None that parked fails. Three drive
past with both, all of them parallel stalls. The car ends nearer to the middle of the stall and
straighter than before: 3.7 cm off sideways on average against 5.5, and 0.4 degrees against
0.9. In depth it does not: 12 cm against 9.

**Seeds 1 to 3, and the stand-in,** are scenarios the method was changed on, so they are no
test of it:

| Perception | Parked | Committed to a stall, and completed | Lateral: mean, worst | Depth: mean, worst | Heading: mean, worst | Smallest clearance | Gear changes: mean, most | Time |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| cameras, seeds 1 to 3 | 42 of 42 | 42 of 42 | 2.7, at most 10.3 cm | 7.4, at most 21.5 cm | 0.4, at most 1.4 deg | 0.18 m | 1.7, 4 | 39 s |
| cameras + lidar, seeds 1 to 3 | 18 of 18 | 18 of 18 | 3.3, at most 14.6 cm | 6.8, at most 12.6 cm | 0.5, at most 1.3 deg | 0.27 m | 1.7, 3 | 38 s |
| stand-in, no sensors | 74 of 74 | 74 of 74 | 2.1, at most 9.7 cm | 4.2, at most 22.6 cm | 0.2, at most 0.8 deg | 0.20 m | 1.4, 4 | 37 s |

The version before parked the same 60 and the same 74. It ended 5.9 cm off sideways on average
with the rig and 5.0 cm with the stand-in, and 1.2 and 0.8 degrees off in heading. Carrying the
position on by the wheels through the maneuver halves both.

The last two changes each concern one kind of stall only: how deep into a stall the car goes
is followed towards the lane for stalls that are not parallel, and not at all for parallel
ones. So the parallel runs of these sets and the others come from two batches, each made with
the code as it now is for its kind. One run was repeated with the final code and gave the
batch's line to the last digit. The earlier seeds, 10 to 27, have not been run again as whole
sets with the final code. Each of their runs that had failed was run again when the change
for it was made, and parked.

**How this version came about.** Seeds 19 to 27 were unseen sets for the version before, three
at a time, and each showed something. Every failure in which the car had committed to a stall
was taken apart: how much room the plan had against the real things, how far the car was from
its path, how far its pose was from the truth.

| On seeds | The version before | What it showed |
| --- | --- | --- |
| 16 to 18 | 43 of 45, completed 43 of 44 | a stall at the end of a row, and the rear camera putting the corner of a van 1.1 m into the stall |
| 19 to 21 | 39 of 45, completed 39 of 42 (with the first four changes below in) | the receiver's position wandering 25 cm towards a kerb during the maneuver, and a car that fell 0.5 m off its path where the curvature reversed. The version before touches in the same three runs |
| 22 to 24 | 39 of 45, completed 39 of 39 | nothing new: six runs drove past |
| 25 to 27 | 42 of 45, completed 42 of 43 | a stall taken 0.9 m too deep when the plan was made |

- *The last stall of a row.* A stub counts as a stall's line if it is one of the lines of a
  stall already made out ([sensors.md](sensors.md#one-line-found-the-other-not)).
- *A single camera at the sides of its image.* Its obstacles are taken from the middle three
  fifths only ([sensors.md](sensors.md#range-from-a-single-camera)).
- *The receiver wanders.* From the moment a stall is chosen the car carries its position on
  by its wheels, along the heading the receiver gives ([sensors.md](sensors.md#limits)).
- *A path the steering cannot follow.* Half of the steering rate is for the path, and the car
  is down to that speed when it gets there ([control.md](control.md#speed-and-torque)).
- *Next to a kerb.* A parallel stall is backed into, with its own allowance for the nose and
  for the tail ([planning.md](planning.md#parallel-parking-the-room-is-on-the-street)).
- *Where a stall begins.* The estimate of the chosen stall is followed across the stall and in
  direction, and along the stall only towards the lane
  ([control.md](control.md#keeping-the-plan-attached-to-the-stall)).
- *Planning again at once.* The monitor asks for the room the plan had, and a goal is moved
  back to the middle of the stall only if the whole path stays clear
  ([control.md](control.md#watching-the-path)).
- *A long way round.* A plan of more than three pieces is held against the plan for the next
  margin ([planning.md](planning.md#configuration-space-by-fft)).

Three of these were got wrong first, and the sets showed it. Carrying the heading on by a
gyro as well was 5 degrees off after a plan that took the car three quarters of the way
round. Not following the stall's depth at all sent the car into the kerb behind a stall that
had been taken 0.9 m too deep. And following a parallel stall towards the street took the car
0.58 m out of it.

**Earlier versions,** on the seeds that were unseen for each:

| Version | Seeds 1 to 3, 60 runs | Seeds 10 to 12, 45 runs | Seeds 13 to 15, 45 runs | Seeds 16 to 18, 45 runs |
| --- | --- | --- | --- | --- |
| before the network and the rows | 49 | 31 (unseen) | not run | not run |
| with the network and the rows | 60 | 36 (unseen) | 38 (unseen) | not run |
| with stalls from one line | 60 | 44 | 42 (unseen) | 43 (unseen) |
| with the kerb side, and no jumps | 60 | 45 | 45 | 43 (unseen) |

Each version was made on what the failures of the one before showed, on the seeds that were
unseen for that one. So a version does well on the seeds it was made on, and the number to go
by is always the one on seeds nobody had looked at.

**The version with the kerb side, and no jumps,** was made on the four failures of seeds 10
to 15.

- *The car touches the kerb beside a parallel stall* (2 runs). The plan swung the nose over a
  kerb that nothing in the map knew of, 0.8 m and 2.0 m beyond where the side of the parked
  car would be.
- *The estimate of the stall jumps, and the car follows it* (1 run). A line of the stall was
  found late, from 2.5 m into the stall, and the stall was rebuilt from there, 0.7 m deeper.
- *The car stops 0.53 m short* (1 run). The kerb behind the stall had been seen only from 11
  to 14 m away and from the side, and a line fitted to what was seen of it came out slanted.

**The version before, with stalls from one line,** was made on the nine failures of seeds 10
to 12 in the same way.

- *The car parks, or is on a good way in, and then shuffles* (4 runs). The estimate of the
  stall moved by a decimetre or two, and the car set out to correct a position that was
  right: up to nine gear changes, and an end position 0.6 m too far out. Since then the car is
  content within 20 cm and 3 degrees, corrects once at most, and holds the estimate over the
  last 2 m ([control.md](control.md#watching-the-path)).
- *The car drives past* (3 runs). One of the two lines of the free stall was never found.
  Such a stall is taken from the one line and its row
  ([sensors.md](sensors.md#one-line-found-the-other-not)).
- *The car gives up* (1 run). A single map cell at the mouth of the chosen stall turned into
  an obstacle while the car stood and planned. It now takes the next stall. Why the cell
  appears is known and not fixed ([sensors.md](sensors.md#limits)).
- *The nose touches a kerb* (1 run). That one was left for this version, above.

On seeds 13 to 15, unseen then, it parked 42 of 45 where the version before it parked 38: four
scenarios more, none fewer.

`python tests/run_set.py cameras`, `python tests/run_set.py cameras --seeds 28,29,30` and
`python tests/run_set.py standin` run these sets. `python docs/run_video.py OUT` with the
arguments of a run makes a video of it: the scene from above with what the car believes drawn
on it. A run repeats exactly, so a run that failed in a set fails the same way for the video.

**Before the network and the rows, and Metal RT.** The 60 runs of seeds 1 to 3 were also
rendered with Metal RT on an Apple M4 Pro, with the version before the network and the rows: 56
of 60 (cameras 39 of 42, cameras and lidar 17 of 18), against 49 of 60 with OptiX. The two renderers give the same scenes with other pixels
([sensors.md](sensors.md#what-it-needs)). They were not run again with Metal RT: on a Mac the
network takes 0.4 s per image, and it is off there unless asked for. One run with it on parked.

What did not park then came from worn paint, in two ways, and this is what the network and
the rows were made for.

- *A stall that is not recognised* (2 of the 4 runs with Metal RT, 9 of the 11 with OptiX). A
  stall between parked cars counted when one of its lines had been found over 2 m and the other
  showed at least its end. One such line is 5.5 m long, with a faded piece 2 m from its start
  and a missing piece further in. It was found over 3.4 m with Metal RT and over 1.9 m with
  OptiX, and that scenario parked with the first and drove past with the second.
- *A stall placed too deep* (2 and 2). The car ended 0.9 m too far into the stall, against the
  kerb. In the one that was looked at, the first metre of one line at the lane was not found,
  and the stall was taken to begin where the paint did.

The offsets are several times those of the earlier version below, where the car knew its pose
exactly. It now parks where it believes it is, and that is 10 cm off.

**Before that: the car told its pose, on clean paint.** Everything from here to the end of this
section is from the version before. There the car was given its true pose, pitch, roll, height
and speed, the lane and the extent of the lot, on a flat road with clean paint
(`--give all --wear 0 --bumps 0`, and a 2 ms simulation step). It shows what the method does
when localization and paint are not the problem.

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
  and on Linux with OptiX, there without a window. The bindings patch was built and checked with
  Vulkan RT in its CPU fallback, but the parking simulation was not run on it. The stand-in
  perception was checked on three PyChrono 10 builds (two conda builds and one from source).
- **Other vehicles.** Only the Chrono sedan. The geometry and the sensor mounts are read from the
  model, so another Chrono vehicle with a hull collision shape and a mesh with glass materials
  should work, but none was tried. One was ruled out early: the BMW E90 model's front wheel flips
  over-centre at full lock in reverse.

## Limits

- **Localization is an error model, not a filter.** The car's pose is the true one with the
  error of a satellite receiver with an inertial unit put on it: 10 cm and 0.3 degrees, wandering
  slowly. No filter runs on simulated GPS, gyro and wheel readings, and the car does not
  localise against the map it builds. Pitch, roll and height do come from what the car sees,
  and speed from a wheel encoder ([sensors.md](sensors.md#limits)).
- **The perception is networks plus rules.** Depth comes from two published networks that were
  not trained on this scene. Lines are found by a rule on brightness and shape, with no learned
  detector. See [sensors.md](sensors.md#limits).
- **Nothing looks sideways, and only the front has two cameras.** Something low that appears
  beside the car is not seen. Behind the car, obstacles are placed only within 1.9 m, and a kerb
  is not seen at all.
- **One exposure.** The cameras do not adapt when the car drives from sun into shade.
- **The world is static.** No moving cars or pedestrians. A new obstacle on the path makes the car
  stop and replan, nothing more.
- **The lane is assumed.** The car takes it that it starts in a lane that runs straight ahead,
  with stalls on either side somewhere in the next 70 m. It does not explore.
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
