"""Rates and speeds of the simulation and the agent."""



STEP = 2e-3                    # simulation step [s]
CONTROL_DT = 0.02              # controller period [s]
PERCEPTION_DT = 0.1            # perception / mapping period [s]
FRAME_MAX = 0.4                # a sensor frame stands for the time since the one before it, at most this [s]

V_SEARCH = 2.2                 # cruise speed while looking for a stall [m/s]
V_FWD = 1.4                    # maneuver speed, forward [m/s]
V_REV = 1.0                    # maneuver speed, reverse [m/s]
STEER_RATE = 0.8               # limit on the road-wheel steering rate [rad/s] (lock to lock in ~1.5 s)
A_DRIVE_MAX = 2.5              # acceleration the drive torque may ask for [m/s^2]
A_BRAKE = 2.5                  # deceleration used to stop [m/s^2]
