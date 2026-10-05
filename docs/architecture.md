# Architecture

This document describes how `parking_sim.py` is put together: the data flow, the rates at which
things run, the state machine that sequences a parking maneuver, and where each part lives in the
file. The other documents go into each stage:

- [sensors.md](sensors.md): the Chrono::Sensor cameras, lidar and radar, and how their data becomes scans and line segments
- [perception-and-mapping.md](perception-and-mapping.md): the stand-in perception, line tracks, occupancy grid, stall inference, stall choice
- [planning.md](planning.md): configuration space, Hybrid A*, Reeds-Shepp curves, docking
- [control.md](control.md): the MPC, the online steering model, speed control, plan re-anchoring
- [simulation-and-viewer.md](simulation-and-viewer.md): the Chrono model, the scenarios, the window
- [results.md](results.md): verification numbers and limits

## The problem

A car drives along a lane next to parking stalls. It knows its own pose. It does not know where the
stalls are, which ones are free, or where the obstacles are. It has to find a free stall, choose
one, and park in it, driving forward and backward as needed.

What it perceives with is selectable. With a sensor rig the car carries a stereo camera that looks
forward and one that looks back, and optionally a lidar or two radars, all ray traced by
Chrono::Sensor. It has no camera to the sides. Without a rig, a stand-in computes noisy line
segments and a noisy range scan from the scenario. Either way the rest of the agent receives the
same two things: planar scans, and segments of painted lines.

The scope is the decision, planning and control problem, with perception from simulated sensors.
Localization is taken as given.

## Data flow

```mermaid
flowchart TB
    subgraph Chrono["Chrono (ground truth)"]
        VEH["Sedan multibody model<br/>tires, suspension, driveline"]
        SCN["Scenario<br/>lines, parked cars, kerbs"]
    end
    subgraph Agent["Parking agent"]
        SENSE["SensorRig.sense or Perception.sense<br/>line segments + planar scans"]
        GRID["GridMap<br/>hits / pass-throughs per cell"]
        LINES["LineMap<br/>one LineTrack per painted line"]
        SLOTS["find_slots<br/>stalls from line pairs"]
        DEC["decide<br/>pick a settled free stall"]
        PLAN["Planner<br/>CSpace + Hybrid A*"]
        TRK["MpcTracker<br/>LateralMPC + SteeringGain + PI"]
    end
    VEH -- "pose" --> SENSE
    SCN --> SENSE
    SENSE -- "scans" --> GRID
    SENSE -- "line detections" --> LINES
    LINES --> SLOTS
    GRID --> SLOTS
    SLOTS --> DEC
    DEC -- "target stall" --> PLAN
    GRID -- "blocked cells" --> PLAN
    PLAN -- "path segments" --> TRK
    SLOTS -. "refined goal pose" .-> TRK
    VEH -- "pose, speed" --> TRK
    TRK -- "steering angle, drive torque, brake torque" --> VEH
```

The agent never reads the scenario directly. Everything it knows about lines and obstacles comes
through `sense`. With a sensor rig that is rendered sensor data. The one thing it takes from
ground truth is its own pose.

## Rates

| Loop | Period | What runs |
| --- | --- | --- |
| Physics | 2 ms | Chrono vehicle and terrain, tire sub-step 1 ms |
| Control | 20 ms | error measurement, steering gain update, MPC solve, speed loop, torque commands |
| Perception and mapping | 100 ms | sensor rendering and processing, grid and line map update, stall inference, decision, plan refinement, path monitor |
| Rendering | about 33 ms | the views of the scene, the sensor pictures and the internals panel |
| Planning | on demand | runs in a worker thread while simulated time is frozen |

`ParkingSim.advance` steps the physics and calls `_perceive` and `_control` on their own
periods. Planning is the only stage that takes noticeable wall time, typically 0.1 to 1 s. While
the planner thread runs, `advance` returns without stepping, so simulated time does not pass and
a run is repeatable regardless of how fast the machine is. The search is limited by a number of
node expansions, not by a wall-clock budget, for the same reason.

## State machine

```mermaid
stateDiagram-v2
    [*] --> SETTLE
    SETTLE --> SEARCH: 1 s, wheels on the ground
    SETTLE --> WAIT: drag mode
    WAIT --> SEARCH: GO, target far down the lane
    WAIT --> BRAKE: GO, target nearby
    SEARCH --> BRAKE: a free stall has settled
    SEARCH --> FAILED: end of the lane, nothing usable
    BRAKE --> PLAN: standing still for 0.3 s
    PLAN --> DRIVE: plan found
    PLAN --> SEARCH: no plan, stall rejected
    PLAN --> FAILED: blocked and no alternative
    DRIVE --> DRIVE: next segment (gear change)
    DRIVE --> BRAKE: stall estimate jumped, or path blocked
    DRIVE --> PLAN: end pose out of tolerance (correction)
    DRIVE --> PARKED: end pose within tolerance
    PARKED --> [*]
    FAILED --> [*]
```

| State | Behaviour |
| --- | --- |
| `SETTLE` | brake held for one second while the suspension settles |
| `SEARCH` | follow the lane at 2.2 m/s, map, look for a stall. With a manual target this state is the approach to it |
| `BRAKE` | brake to a stop. Planning always starts from standstill |
| `PLAN` | planner thread running, time frozen |
| `DRIVE` | track the plan one segment at a time. Each segment is driven in one direction and ends with a full stop |
| `PARKED`, `FAILED` | brake held, wheels straightened, result printed |
| `WAIT` | drag mode only: the car stands still, keeps mapping what it can see, and waits for a target |

Three events can interrupt `DRIVE`:

1. The estimate of the target stall moves by more than 0.5 m or 0.1 rad. The car stops and replans.
   Smaller movements do not interrupt anything. They shift the end of the plan gradually (see
   [control.md](control.md#keeping-the-plan-attached-to-the-stall)).
2. Newly seen obstacle cells lie inside the footprint along the remaining path, on two consecutive
   perception ticks. The car stops and replans. If no plan exists, the run fails instead of driving
   a blocked path. With a sensor rig the footprint is grown by a margin for this test, because a
   camera places an obstacle exactly only once it is close
   ([sensors.md](sensors.md#the-monitor-looks-for-margin)).
3. After the last segment, the pose is more than 0.08 m sideways, 1.5 degrees, or 0.3 m lengthwise
   from the goal. The car plans a correction, at most twice.

## Coordinates and conventions

- World frame: x east, y north, z up. All planning is planar.
- The planner and the controller describe the car by the pose `(x, y, theta)` of the centre of
  its **rear axle**. That is the point at which the kinematic bicycle model has no sideslip, in
  both driving directions.
- Curvature `kappa` is signed in the car's frame: positive means steering left, in forward and
  in reverse. With signed speed `v`, the yaw rate is `v * kappa`.
- Direction `d` is +1 forward and -1 reverse.
- The three commands to the car are physical: steering angle `delta` (road-wheel angle, rad,
  positive left), drive torque at the wheels (N m, negative drives backwards) and brake torque
  (N m). Chrono's normalized pedal inputs are not used by the agent.

## Where things are in the file

`parking_sim.py` is a single file on purpose, so it can be dropped next to any PyChrono
installation and run. It is organised in sections, in this order:

| Section | Main names |
| --- | --- |
| Ego vehicle | `Ego`, `EGO` (geometry, mass and limits read from the Chrono model) |
| Geometry helpers | `rect_poly`, `ego_poly`, `poly_distance`, `footprint_hits` |
| Scenarios | `Scenario`, `make_lot`, `make_street`, `parked_model` |
| Perception | `Perception` (stand-in), `SensorRig`, `planar_scan`, `paint_segments` (Chrono::Sensor) |
| Mapping | `GridMap`, `LineTrack`, `LineMap` |
| Stall inference | `Slot`, `find_slots`, `_classify`, `_align_with_kerb` |
| Reeds-Shepp | `_rs_words`, `rs_paths`, `rs_length_table`, `rs_sample` |
| Configuration space | `CSpace`, `holonomic_distance` |
| Planner | `Planner` (`search`, `shoot`, `_rs_shot`, `_arc_shot`), `Segment`, `split_segments` |
| Control | `LateralMPC`, `SteeringGain`, `MpcTracker` |
| Chrono world | `World` (the model, the scene, and the physical actuation in `step`) |
| Agent | `ParkingSim` (state machine, decision, planning requests, refinement, monitor) |
| Viewer | `MouseKeys`, `Viewer` |
| Entry point | `parse_args`, `main` |

## Design choices worth knowing

**Unseen space is blocked.** The planner only drives through cells that a range ray has passed
through. The far side of a parked car was never seen to be free, so it is solid, even though only
its near face produced range returns. This removes a whole class of plans that would cut through
an obstacle the car has only seen one side of. With cameras that look only forward and back, a lot
is unseen at any moment, and the map is what carries the car past it.

**Plans end with a straight docking run.** The search does not aim at the parked pose. It aims at a
point on the stall axis a few metres short of it, and the last stretch is a straight line along the
axis. Whatever error is left after the turning part gets removed there, which is where the final
accuracy comes from.

**Each segment ends in a stop.** A plan is cut at every direction change. The car stops, changes
gear, turns the wheels to where the next segment needs them, and only then moves. Steering while
standing is slow on a real car, but it removes the transient at the start of every segment.

**The plan follows the stall, not the other way round.** The stall estimate keeps improving as the
car gets closer. The remaining part of the plan is moved rigidly with it, weighted so that only the
last few metres move. The car therefore ends up where the lines are, not where they were first
believed to be.

**No measured vehicle data.** Geometry, mass, wheel radius, steering stop and brake capacity are
read from the Chrono model at start-up. The planner uses the kinematic curvature limit that follows
from the model's wheelbase and declared maximum steering angle. The controller starts from the
ideal bicycle model and identifies the real steering gain while driving (see
[control.md](control.md#the-steering-gain)).

**Physical commands.** The controller outputs a steering angle, a drive torque and a brake torque,
and the simulation applies those to the model directly (see
[simulation-and-viewer.md](simulation-and-viewer.md#actuation)). There is no throttle map, gearbox
or steering ratio hidden between the controller and the car.
