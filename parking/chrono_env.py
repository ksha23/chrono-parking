"""PyChrono, found and imported. If this Python has none, the script is rerun with one that has."""

import glob
import os
import sys

__all__ = ["chrono", "veh", "sens", "HAVE_SENSORS"]


def _conda_roots():
    roots = []
    for var in ("CONDA_EXE", "MAMBA_EXE"):
        exe = os.environ.get(var)
        if exe:
            roots.append(os.path.dirname(os.path.dirname(exe)))
    roots += ["/opt/homebrew/Caskroom/miniconda/base", "/opt/homebrew/anaconda3",
              os.path.expanduser("~/miniconda3"), os.path.expanduser("~/anaconda3"),
              os.path.expanduser("~/miniforge3"), os.path.expanduser("~/mambaforge")]
    return list(dict.fromkeys(roots))


def _reexec_in_chrono_env():
    """PyChrono is not importable here: look for a conda env that has it."""
    if os.environ.get("PARKING_SIM_REEXEC"):
        return
    found = []
    for root in _conda_roots():
        for f in glob.glob(os.path.join(root, "envs", "*", "lib", "python*", "site-packages",
                                        "pychrono", "vehicle.py")):
            if os.path.exists(os.path.join(os.path.dirname(f), "irrlicht.py")):
                env = f.split(os.sep + "lib" + os.sep)[0]
                py = os.path.join(env, "bin", "python")
                if os.path.exists(py):
                    found.append((os.path.getmtime(f), py))
    if found:
        py = max(found)[1]
        print("[parking] PyChrono not found in this interpreter, re-running with %s" % py)
        os.environ["PARKING_SIM_REEXEC"] = "1"
        os.execv(py, [py] + sys.argv)


try:
    import pychrono as chrono
    import pychrono.vehicle as veh
except ImportError:
    _reexec_in_chrono_env()
    sys.exit("PyChrono with the vehicle and irrlicht modules is required (see docs/ for the setup)")
try:
    import pychrono.sensor as sens
except ImportError:
    sens = None
# Cameras and lidar need a PyChrono whose sensor module wraps the ray-traced sensors
HAVE_SENSORS = sens is not None and hasattr(sens, "ChCameraSensor")
