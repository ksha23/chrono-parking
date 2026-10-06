# chrono-parking

Automated parking in [Project Chrono](https://projectchrono.org), in Python.

A full multibody Chrono::Vehicle sedan drives down a parking aisle. From what its sensors show
it builds a map of the painted stall lines and the obstacles, decides which stall to take, plans
a forward and reverse maneuver into it, and tracks that plan with model predictive control. It
handles perpendicular, angled and parallel stalls, with cars on one side, both sides or none.

The car perceives with what a production car could carry: a stereo pair of cameras behind the
windshield, one camera at the tail, one on the front bumper, and optionally a forward-facing
lidar. Nothing looks sideways. The cameras are simulated by Chrono::Sensor as Stereolabs
ZED X One units, with sensor noise, a sun that casts shadows over worn paint, and a textured
road. There is no depth camera: depth is computed from the images, by the IGEV++ stereo network
for the pair and by a monocular depth network for the single cameras. On a PyChrono without
ray-traced sensors, a stand-in computes noisy detections from the scenario instead.

![The simulator window](docs/img/parking.gif)

*`python parking_sim.py --sensors camera+lidar`*

The window shows three things. On the left, the scene from above and from behind the car, with
what the car knows drawn in. Next to it, what the sensors deliver and what is computed from it:
the left image of the stereo pair, the depth that IGEV++ makes of the pair, the rear and the
bumper image, and every range from above, with the lidar's range image. On the right, a panel
with the internals: which stage of the pipeline is working, the planning map with the
Hybrid A* search tree, the MPC horizon, tracking error, speed, the steering gain the controller is
identifying as it drives, and what each camera's image is being read as.

## Run it

You need a Python that has PyChrono with its `vehicle` and `irrlicht` modules, plus NumPy. Nothing
else. See the [PyChrono installation guide](https://api.projectchrono.org/pychrono_installation.html).
If the interpreter you use has no PyChrono, the script looks for a conda environment that does and
relaunches itself there.

The sensors need two more things, both described in
[docs/sensors.md](docs/sensors.md#what-it-needs). Without them the script runs with
`--sensors sim`.

- A PyChrono whose `sensor` module has cameras and lidar. A Chrono built with OptiX has that. A
  Chrono built with Metal RT (macOS) or Vulkan RT needs a small patch to its Python bindings,
  which is in this repository. Everything here was developed with Metal RT. It also runs with
  OptiX on Linux, there without a window.
- A Python with PyTorch for the networks, which run in a process of their own
  (`parking/stereo_worker.py`), and a checkout of IGEV++ with its published weights. The other
  two networks are fetched when first used: Depth Anything V2 for the single cameras, and
  Mask2Former for what is in an image (markings, kerbs, the car's own body). The weights of the
  last one are for non-commercial use.

```
python parking_sim.py                                   # perpendicular stalls, a car on each side
python parking_sim.py --sensors camera+lidar            # cameras and a forward-facing lidar
python parking_sim.py --stereo rt --sky overcast        # the faster stereo network, a grey day
python parking_sim.py --sensors sim                     # no sensors: detections from the scenario
python parking_sim.py --type angled --cars none         # 60 degree stalls, empty lot, lines only
python parking_sim.py --type parallel --cars both       # parallel park between two cars
python parking_sim.py --type perpendicular --park forward
python parking_sim.py --tour                            # eight scenarios back to back
python parking_sim.py --target drag                     # place the target yourself (macOS)
python parking_sim.py --headless --seed 7 --noise 2     # no window, prints a result line
```

| Option | Values | Meaning |
| --- | --- | --- |
| `--sensors` | `camera`, `camera+lidar`, `sim` | what the car perceives with. Default: `camera` if the PyChrono has the sensors and the depth networks are set up, else `sim` |
| `--stereo` | `igev`, `rt` | stereo network: IGEV++, or its real-time version, three times faster and less exact (see [docs/sensors.md](docs/sensors.md#what-it-needs)) |
| `--stereo-rows` | `TOP,BOTTOM`, for example `160,544` | give the stereo network only these rows of the 600 of an image, counted from the top: above is sky, below is the car's own bonnet. Default: all rows |
| `--stereo-hz`, `--mono-hz` | per second, default 5 | how often the stereo network and the monocular network run, up to the 10 frames per second of the cameras. The maps add up time, not frames, so nothing is tuned to a rate: see [docs/sensors.md](docs/sensors.md#how-often-the-networks-run) |
| `--sky` | `clear`, `low`, `overcast` | light for the sensors: sun at 41 degrees, sun at 32 degrees, or a grey sky. Default: by the seed |
| `--pose-noise` | scale, default 1 | how well the car knows where it is. At 1 its position is off by a slowly wandering 10 cm and its heading by 0.3 degrees, like a satellite receiver with an inertial unit. `--pose odometry` gives dead reckoning instead, which drifts |
| `--bumps` | cm, default 1.5 | how uneven the road is: up to this much up and down, in waves 6 to 25 m long |
| `--give` | any of `pose`, `attitude`, `speed`, `lane`, `map`, or `all` | tell the car things a real car would not know, to tell causes apart. Default: nothing. See [docs/sensors.md](docs/sensors.md#limits) |
| `--wear` | scale, default 1 | worn paint: every line is patchy with ragged edges, one in four is faded and one in twelve is barely lighter than the road. 0 = clean bars, as the results below were measured with |
| `--scene` | `auto`, `net`, `none` | a network that labels each image points out faint paint, kerbs and the car's own bonnet. `auto` turns it on where the networks run on a CUDA GPU, see [docs/sensors.md](docs/sensors.md#what-it-needs) |
| `--depth-host` | ssh host | run the depth networks on another machine, see [docs/sensors.md](docs/sensors.md#what-it-needs) |
| `--type` | `perpendicular`, `angled`, `parallel` | kind of stalls |
| `--cars` | `both`, `left`, `right`, `none`, `random` | parked cars next to the free stall, seen from the lane looking into it |
| `--side` | `right`, `left` | which side of the lane the free stall is on |
| `--angle` | degrees, default 60 | stall angle for `--type angled` |
| `--park` | `auto`, `forward`, `reverse` | nose in or back in |
| `--target` | `drag` or `X,Y,DEG` | choose the spot yourself instead of letting the car choose |
| `--tire` | `tmeasy`, `pac02` | tire model of the simulated car |
| `--noise` | scale, default 1 | perception noise, 0 is perfect |
| `--seed` | integer | layout details and noise |
| `--headless` | | no window, as fast as possible |
| `--layout` | `sensors`, `quad`, `wide` | sensor pictures next to two views of the scene (default with sensors), or four views of the scene |
| `--no-panel` | | hide the internals panel |
| `--snapshots DIR` | | save window frames as PNG |

`python parking_sim.py --help` lists the rest.

## How it works

```mermaid
flowchart LR
    V["Chrono::Vehicle sedan<br/>2 ms step"] -->|true pose| P["Perception<br/>Chrono::Sensor cameras, depth networks<br/>10 Hz"]
    P --> M["Mapping<br/>line tracks + occupancy grid"]
    M --> D["Stall inference<br/>and choice"]
    D --> PL["Planning<br/>Hybrid A* + Reeds-Shepp"]
    PL --> C["Control<br/>MPC steering + PI speed<br/>50 Hz"]
    C -->|"steering angle, drive torque, brake torque"| V
    M -. "stall estimate keeps improving" .-> C
```

| Stage | What it does | Details |
| --- | --- | --- |
| Perception | painted lines from the camera images, obstacles and free ground from depth that neural networks compute from those images, optionally a lidar. Or a stand-in without sensors | [docs/sensors.md](docs/sensors.md) |
| Mapping | weighted total least squares line tracks, occupancy grid where unseen space counts as blocked | [docs/perception-and-mapping.md](docs/perception-and-mapping.md) |
| Decision | stalls from pairs of lines, free or occupied from the map, first settled free stall wins | [docs/perception-and-mapping.md](docs/perception-and-mapping.md) |
| Planning | configuration space by FFT, Hybrid A*, Reeds-Shepp and arc-line analytic expansions | [docs/planning.md](docs/planning.md) |
| Control | linear MPC in the distance domain, constrained QP solved exactly, steering gain identified online, commands in physical units | [docs/control.md](docs/control.md) |
| Simulation | Chrono sedan, scenario generator, viewer, target placement by mouse | [docs/simulation-and-viewer.md](docs/simulation-and-viewer.md) |

Start with [docs/architecture.md](docs/architecture.md) for the overall structure and the state
machine. [docs/results.md](docs/results.md) has the verification numbers and the known limits.

## What is simulated

The car is the Chrono `Sedan` model: 20 rigid bodies, double wishbone front and multi-link rear
suspension, rack and pinion steering, a shaft-based driveline and brakes, and TMeasy or Pacejka
tires. That is a full multibody vehicle, a superset of the usual 14 degree of freedom model. The
kinematic bicycle model appears only inside the planner and the MPC.

The car is driven by physical commands, not pedal positions: a road-wheel steering angle in
radians, a drive torque at the wheels and a brake torque in newton metres. The drive torque goes
straight onto the half-shafts of the driven axle, with the gearbox in neutral.

Nothing about the car is hard-coded. Its wheelbase, body outline, mass, wheel radius, steering
stop and brake capacity are read from the Chrono model at start-up, and its steering response is
identified online while it drives.

## Results

A run counts as parked when the car ends inside the lines of a free stall without having touched
anything. All offsets are measured against the ground-truth stall.

The car is told only what a real car would know: a pose with the error of a satellite receiver
with an inertial unit (10 cm and 0.3 degrees), its speed from a wheel encoder, its pitch, roll
and height from the road it sees, and that it starts in a lane with stalls along it. The paint
is worn and the road is uneven. Rendered with OptiX:

| Perception | Parked, unseen seeds 13 to 15 | Parked, seeds 10 to 12 | Parked, seeds 1 to 3 | Lateral offset: mean, worst | Heading error: mean, worst | Smallest clearance |
| --- | --- | --- | --- | --- | --- | --- |
| cameras: stereo pair, rear, bumper | 35 of 36 | 35 of 36 | 42 of 42 | 5.5, at most 18.9 cm | 1.3, at most 3.4 deg | 0.14 m |
| cameras + forward lidar | 7 of 9 | 9 of 9 | 18 of 18 | 4.1, at most 9.9 cm | 0.9, at most 2.1 deg | 0.15 m |
| stand-in, no sensors | | | 74 of 74 | 5.0, at most 14.6 cm | 0.8, at most 2.5 deg | 0.08 m |

Seeds 1 to 3 and 10 to 12 are scenarios the method was changed on. Seeds 13 to 15 had never
been run: 42 of 45 is the number to go by. (The offsets of the rig are those of the unseen
seeds.) The version before parked 38 of the same 45, and 36 of 45 on seeds 10 to 12 when those
were unseen. Its failures there were looked at on video, and three things were changed: the car
no longer shuffles after a stall estimate that moves by a decimetre, a stall of which only one
line was found is taken from that line and its row, and a stall that turns out to be blocked
while the car is still on the lane is given up for the next one. What still does not park on
the unseen seeds: the estimate of a stall jumps while the car drives in and the car follows
it (1 run), the car touches the kerb of a parallel stall (1), the car stops 0.5 m short of a
kerb it places too near (1). The [results](docs/results.md) have the list.

**Before that**, the car was given its true pose, pitch, roll, height and speed, the lane and
the extent of the lot, on a flat road with clean paint (`--give all --wear 0 --bumps 0`, and a
2 ms simulation step). The numbers of that version:

| Perception | Parked, seeds 1 to 3 | Seeds 4 to 6 | Unseen seeds 7 to 9 | Lateral offset | Heading error | Smallest clearance |
| --- | --- | --- | --- | --- | --- | --- |
| cameras: stereo pair, rear, bumper | 42 of 42 | 35 of 36 | 36 of 36 | at most 6.0 cm | at most 1.20 deg | 0.17 m |
| cameras + forward lidar | 18 of 18 | 9 of 9 | 9 of 9 | at most 6.7 cm | at most 0.94 deg | 0.15 m |
| stand-in, no sensors | 74 of 74 | | | at most 2.8 cm | at most 1.50 deg | 0.11 m |


The rig was run on perpendicular, angled and parallel stalls with cars on both sides, one side
or none, under a clear sky, a low sun and an overcast sky, and at double sensor noise. Seeds 1 to
6 are runs the method was fixed on: each batch that missed a run led to a change, the last of
them from counting camera frames to adding up time, so that nothing is tuned to how often the
networks run. Seeds 7 to 9 had never been run and were made once with the final code. The one
run that did not park stopped in mid-manoeuvre with no way on, 1.1 m from anything: the stereo
network had put the edge of the car's own bonnet into the map as an obstacle while the car stood
still. The stand-in was run on the 74 scenarios of the earlier versions. A run with the rig takes
two to four minutes, most of it in the stereo network, which is why it has fewer runs, and it
can be repeated exactly. [docs/results.md](docs/results.md) has every row and the limits, and
[docs/sensors.md](docs/sensors.md#how-good-it-is) measures the perception itself: range,
obstacles and lines against the geometry of the scenario.

## Layout

```
parking_sim.py                  entry point: python parking_sim.py [options]
parking/
  cli.py                        options, the tour, the main loop
  agent.py                      the parking agent: one state machine from search to parked
  chrono_env.py                 finds and imports PyChrono
  config.py, vehicle.py         rates and speeds, the car as read from the Chrono model
  geometry.py                   footprints and distances
  scenario.py, world.py         the lot and its ground truth, the Chrono world built from it
  ground.py, paint.py           the uneven road, the worn paint of the lines
  localization.py               the pose the car believes it has, its wheel encoder
  perception.py                 the stand-in perception, planar scans, line segments from paint
  sensors.py                    the sensor rig: cameras, lidar, depth from the images
  stereo_worker.py              the networks (IGEV++, Depth Anything V2, Mask2Former), a process of their own
  networks.py                   starting that process and talking to it
  scene_net.py                  the network that labels markings, kerbs and the car's own body
  mapping.py, stalls.py         occupancy grid and line tracks, stalls inferred from pairs of lines
  slot.py                       a stall, and what the map says about it: free or taken, neighbours, kerb
  rows.py, one_line.py          what the stalls on one side of the lane have in common, and stalls taken from that
  reeds_shepp.py, planner.py    Reeds-Shepp curves, configuration space, Hybrid A*
  control.py                    steering MPC, online steering gain, speed control
  viewer.py                     the window, with viewer_pictures.py, viewer_panel.py, draw.py, inputs.py
tests/test_core.py              checks of the planner curves, the MPC solver, the image processing and the map
tests/run_set.py                runs a set of scenarios a few at a time and says what parked
docs/                           design documents and figures
docs/make_figures.py            regenerates the figures of the pipeline from real runs
docs/make_sensor_figures.py     regenerates the sensor figures from real runs and measures the perception
docs/rate_check.py              replays recorded drives at several network rates: the choice of stall must not depend on the rate
docs/run_video.py               a video of a run: the scene from above with what the car believes drawn on it
docs/pychrono-rt-sensors.patch  Python bindings for Chrono's ray-traced sensors with Metal RT and Vulkan RT
```
