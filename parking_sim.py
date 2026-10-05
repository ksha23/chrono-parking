#!/usr/bin/env python3
# =============================================================================
# Automated parking in Project Chrono (PyChrono)
#
# A Chrono::Vehicle sedan (full multibody model, TMeasy or Pacejka tires) cruises
# down a parking aisle, builds a map of the stall lines and obstacles from what
# its sensors show, decides which stall to take, plans a forward/reverse
# maneuver into it and tracks that plan with model predictive control. One
# window shows the scene, what the sensors deliver and a panel with the internals.
#
#   perception : Chrono::Sensor cameras where a production car has them: a stereo
#                pair behind the windshield, one at the tail, one on the front
#                bumper, optionally a forward-facing lidar. Depth is computed
#                from the images by neural networks (IGEV++ for the pair), not
#                read from the renderer. No camera looks sideways. Without
#                ray-traced sensors: noisy detections computed from the scenario
#   mapping    : line tracks (total least squares) + occupancy grid
#   decision   : stalls are inferred from pairs of tracked lines, classified
#                free / occupied / unknown, and scored
#   planning   : Hybrid A* with Reeds-Shepp and arc-line analytic expansions
#   control    : linear MPC on the steering (constrained QP, solved exactly) with
#                the steering gain identified online, and a PI loop on speed
#
# The car is driven by physical commands, not pedal positions: road-wheel steering
# angle [rad], drive torque at the wheels [N m] and brake torque [N m]. Nothing
# about the car is hard-coded: its geometry, mass, actuator limits and the places
# where the sensors are mounted are read from the Chrono model at start-up. The
# design is documented in docs/.
#
# This file is only the entry point. The code is in the package parking/, one
# module per concern: docs/architecture.md says what is where.
#
# Examples (any Python with PyChrono's vehicle and irrlicht modules; the sensors
# also need its sensor module with cameras and lidar, and a Python with PyTorch
# for the depth networks, see docs/sensors.md):
#
#   python parking_sim.py                                  perpendicular, car on each side
#   python parking_sim.py --sensors camera+lidar           cameras and a forward-facing lidar
#   python parking_sim.py --stereo rt                      the faster stereo network
#   python parking_sim.py --sensors sim                    no sensors, detections from the scenario
#   python parking_sim.py --layout quad                    four views of the scene, no sensor pictures
#   python parking_sim.py --type angled --cars none        60 deg stalls, empty lot
#   python parking_sim.py --type parallel --cars both      parallel park between two cars
#   python parking_sim.py --type perpendicular --cars left --park forward
#   python parking_sim.py --tour                           a set of scenarios back to back
#   python parking_sim.py --target drag                    place the target box yourself
#   python parking_sim.py --headless --seed 7 --noise 2    no window, prints the result
#
# "python parking_sim.py --help" lists every option.
# =============================================================================

import sys

from parking.cli import main

if __name__ == "__main__":
    sys.exit(main())
