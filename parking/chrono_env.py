"""PyChrono, found and imported. If this Python has none, the script is rerun with one that has."""

import glob
import os
import sys

__all__ = ["chrono", "veh", "sens", "HAVE_SENSORS", "rerun_with_own_build"]


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


def _own_build():
    """A PyChrono built from source, if one was named: (its Python, the directory to put on the
    path). It is named with PARKING_PYCHRONO or linked as third_party/pychrono, and is the
    directory of the build that holds the pychrono package (build/bin). The Python the build was
    made for is read from the build itself."""
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    for d in (os.environ.get("PARKING_PYCHRONO"), os.path.join(here, "third_party", "pychrono")):
        d = os.path.realpath(os.path.expanduser(d)) if d else ""
        if not os.path.exists(os.path.join(d, "pychrono", "__init__.py")):
            continue
        py, cache = None, os.path.join(os.path.dirname(d), "CMakeCache.txt")
        if os.path.exists(cache):
            with open(cache) as f:
                for line in f:
                    if line.startswith("Python3_EXECUTABLE:") and "=" in line:
                        py = line.split("=", 1)[1].strip()
        return (py if py and os.path.exists(py) else sys.executable), d
    return None


def rerun_with_own_build(why):
    """Start again with the PyChrono built from source, if one was named and this is not it."""
    build = _own_build()
    if build is None or os.environ.get("PARKING_SIM_REEXEC"):
        return
    py, path = build
    print("[parking] %s, re-running with the PyChrono in %s" % (why, path), flush=True)
    os.environ["PARKING_SIM_REEXEC"] = "1"
    os.environ["PYTHONPATH"] = os.pathsep.join([path] + [q for q in os.environ.get("PYTHONPATH", "").split(os.pathsep) if q])
    os.execv(py, [py] + sys.argv)


def _reexec_in_chrono_env():
    """PyChrono is not importable here: use the build that was named, or look for a conda env
    that has it."""
    rerun_with_own_build("PyChrono not found in this interpreter")
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
        print("[parking] PyChrono not found in this interpreter, re-running with %s" % py, flush=True)
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
