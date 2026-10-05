"""Command line: options, the tour, and the main loop with or without the window."""

import argparse
import math
import os
import subprocess
import sys

from .agent import ParkingSim
from .chrono_env import HAVE_SENSORS, rerun_with_own_build, sens
from .localization import Localization
from .sensors import SensorRig, find_depth_python
from .world import SKIES


TOUR = [("perpendicular", "both"), ("perpendicular", "left"), ("perpendicular", "right"),
        ("perpendicular", "none"), ("angled", "both"), ("angled", "none"), ("parallel", "both"),
        ("parallel", "none")]


def parse_args(argv=None):
    ap = argparse.ArgumentParser(
        description="Automated parking in Project Chrono: perpendicular, angled and parallel stalls.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--type", choices=("perpendicular", "angled", "parallel"), default="perpendicular",
                    help="kind of parking stalls")
    ap.add_argument("--cars", choices=("both", "left", "right", "none", "random"), default="both",
                    help="parked cars next to the free stall, as seen from the driving lane "
                         "looking into the stall (none = empty lot, lines only)")
    ap.add_argument("--side", choices=("right", "left"), default="right",
                    help="side of the lane the free stall is on")
    ap.add_argument("--angle", type=float, default=60.0, help="stall angle for --type angled [deg]")
    ap.add_argument("--park", choices=("auto", "forward", "reverse"), default="auto",
                    help="drive into perpendicular/angled stalls nose first or backwards")
    ap.add_argument("--target", default=None, metavar="drag | X,Y,DEG",
                    help="choose the spot yourself: 'drag' to place a box with the mouse in the top "
                         "view, or the pose of the middle of the car")
    ap.add_argument("--no-snap", action="store_true",
                    help="with --target: park exactly where the box is, do not align with a detected stall")
    ap.add_argument("--tire", choices=("tmeasy", "pac02"), default="tmeasy",
                    help="tire model of the simulated sedan (TMeasy or Pacejka 2002)")
    ap.add_argument("--sensors", choices=("auto", "sim") + SensorRig.MODES, default="auto",
                    help="what the car perceives with. camera: Chrono::Sensor cameras, a stereo pair behind the "
                         "windshield, one at the tail and one on the front bumper, with depth computed from the "
                         "images by neural networks. camera+lidar: the same plus a forward-facing lidar. sim: "
                         "detections computed from the scenario, no sensor simulated. auto (default): camera if "
                         "this PyChrono has the ray-traced sensors and the networks are set up, otherwise sim")
    ap.add_argument("--stereo", choices=("igev", "rt"), default="igev",
                    help="stereo network: IGEV++ or its real-time version, which is three times faster and "
                         "a little less accurate")
    ap.add_argument("--stereo-hz", type=float, default=5.0,
                    help="how often the stereo network runs, up to the 10 frames per second of the cameras. "
                         "The default is what IGEV++ reaches in real time on a desktop GPU. The method does not "
                         "count frames, so nothing has to be retuned for another rate")
    ap.add_argument("--mono-hz", type=float, default=5.0,
                    help="how often the monocular network runs on the single cameras")
    ap.add_argument("--stereo-rows", default=None, metavar="TOP,BOTTOM",
                    help="give the stereo network only these rows of the 600 of an image, counted from the top, "
                         "for example 160,544: above is sky, below is the car's own bonnet. The network is "
                         "faster by about the share of rows left out. Default: all rows")
    ap.add_argument("--sky", choices=("auto",) + tuple(SKIES), default="auto",
                    help="light for the sensors: a clear sky with the sun at 41 degrees, a low sun at 32 degrees, "
                         "or an overcast sky. auto picks by the seed")
    ap.add_argument("--depth-python", default=None, metavar="PYTHON",
                    help="the Python that runs the depth networks (default: one that has torch, timm and transformers)")
    ap.add_argument("--depth-host", default=None, metavar="HOST",
                    help="run the depth networks on another machine, reached with ssh. It needs a copy of this "
                         "repository with third_party/IGEV-plusplus, and --depth-python names its Python")
    ap.add_argument("--depth-dir", default="chrono-parking", metavar="DIR",
                    help="where that copy is on the other machine")
    ap.add_argument("--igev", default=None, metavar="DIR",
                    help="checkout of the IGEV++ repository with its weights (default: third_party/IGEV-plusplus)")
    ap.add_argument("--noise", type=float, default=1.0, help="perception noise scale (0 = perfect)")
    ap.add_argument("--wear", type=float, default=1.0,
                    help="how worn the paint is. At 1 (default) every line is patchy and ragged, one in four is "
                         "faded and one in twelve is barely lighter than the road. Between 0 and 1 fewer are "
                         "faded. 0 = clean bars of flat grey, as the results in the docs were measured with")
    # The next three make the world less kind as well. They are off by default until the whole
    # set of scenarios has been run with them.
    ap.add_argument("--bumps", type=float, default=0.0, metavar="CM",
                    help="how uneven the road is: it rises and falls by up to this much, in waves 6 to 25 m long "
                         "(try 1.5). 0 (default) = a perfect plane")
    ap.add_argument("--pose", choices=Localization.SOURCES, default="gps",
                    help="where the car gets its own pose from: a satellite receiver with an inertial unit, "
                         "whose error wanders slowly, or dead reckoning from wheel speed and yaw rate, "
                         "whose error grows as it drives")
    ap.add_argument("--pose-noise", type=float, default=0.0,
                    help="scale of the errors of that pose (at 1: 10 cm and 0.3 degrees with gps) and of the pitch "
                         "and roll the car assumes (0.15 degrees). 0 (default) = it knows its true pose")
    ap.add_argument("--seed", type=int, default=1, help="random seed (layout details and noise)")
    ap.add_argument("--tour", action="store_true", help="play through a set of scenarios one after another")
    ap.add_argument("--headless", action="store_true", help="no window, run as fast as possible")
    ap.add_argument("--layout", choices=("sensors", "quad", "wide"), default=None,
                    help="sensors: two views of the scene next to the images and range data that the sensors "
                         "deliver (default with simulated sensors). quad: 2x2 views of the scene. wide: a large "
                         "top view with three small ones (default with --target drag and no sensors)")
    ap.add_argument("--window", default="1600x930", help="window size")
    ap.add_argument("--no-panel", action="store_true", help="hide the internals panel next to the views")
    ap.add_argument("--speed", type=float, default=1.0, help="playback speed relative to real time")
    ap.add_argument("--exit-after", type=float, default=None,
                    help="close the window this many seconds after parking (default: stay open)")
    ap.add_argument("--snapshots", default=None, metavar="DIR", help="save a PNG of the window every --snapshot-dt")
    ap.add_argument("--snapshot-dt", type=float, default=2.0, help="simulated time between snapshots [s]")
    ap.add_argument("--timeout", type=float, default=240.0, help="give up after this much simulated time [s]")
    args = ap.parse_args(argv)
    args.window = tuple(int(v) for v in args.window.lower().split("x"))
    if args.target is not None and args.target != "drag":
        x, y, deg = (float(v) for v in args.target.split(","))
        args.target = (x, y, math.radians(deg))
    if args.stereo_rows is not None:
        try:
            top, bottom = (int(v) for v in args.stereo_rows.split(","))
        except ValueError:
            ap.error("--stereo-rows takes two row numbers, like 160,544")
        if not 0 <= top < bottom <= SensorRig.CAM_H or bottom - top < 64:
            ap.error("--stereo-rows: rows are 0 to %d from the top, and at least 64 of them are needed" % SensorRig.CAM_H)
        args.stereo_rows = (top, bottom)
    if args.snapshots:
        os.makedirs(args.snapshots, exist_ok=True)
    if args.sky == "auto":
        args.sky = tuple(SKIES)[(args.seed - 1) % len(SKIES)]
    if args.sensors == "auto":
        args.sensors = "camera" if HAVE_SENSORS and (args.depth_host or find_depth_python(args.depth_python)) else "sim"
    elif args.sensors != "sim" and not HAVE_SENSORS:
        rerun_with_own_build("this PyChrono has no cameras")
        ap.error("--sensors %s needs a PyChrono whose sensor module has cameras and lidar (a build "
                 "with a ray-tracing backend and Python bindings for it, see docs/sensors.md). This one has %s. "
                 "If you have such a build, name its bin directory in PARKING_PYCHRONO or link it as "
                 "third_party/pychrono. Use --sensors sim to run without simulated sensors." %
                 (args.sensors, "no sensor module" if sens is None else "only GPS and IMU sensors"))
    if args.layout == "sensors" and args.sensors == "sim":
        ap.error("--layout sensors shows the output of simulated sensors, and --sensors sim has none")
    if args.layout is None:
        args.layout = "sensors" if args.sensors != "sim" else "wide" if args.target == "drag" else "quad"
    return args


def main():
    args = parse_args()
    if args.tour:
        rest = [a for a in sys.argv[1:] if a != "--tour"]
        for kind, cars in TOUR:
            print("\n=== %s stalls, cars: %s ===" % (kind, cars), flush=True)
            code = subprocess.call([sys.executable, os.path.abspath(sys.argv[0])] + rest +
                                   ["--type", kind, "--cars", cars, "--exit-after", "4"])
            if code not in (0, 1):
                return code
        return 0
    sim = ParkingSim(args)
    if args.headless:
        if args.target == "drag":
            sys.exit("--target drag needs the window; use --target X,Y,DEG with --headless")
        while sim.result is None and sim.time < args.timeout:
            sim.advance(500)
        if sim.result is None:
            sim._finish(False, "timed out")
        sys.stdout.flush()               # leave without running destructors, as below: the sensor
        sys.stderr.flush()               # manager owns render threads
        os._exit(0 if sim.result["ok"] else 1)
    else:
        from .viewer import Viewer          # only now: a headless run needs no Irrlicht
        viewer = Viewer(sim, args)
        code = 1
        try:
            viewer.loop()
            code = 0 if sim.result is not None and sim.result["ok"] else 1
        except BaseException:
            import traceback
            traceback.print_exc()
        # Destroying the Irrlicht visual system from Python segfaults here (after the window
        # has closed), so keep it alive and leave without running destructors
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(code)
    return 0 if sim.result is not None and sim.result["ok"] else 1
