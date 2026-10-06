"""The process that runs the networks, as the simulation sees it: start it, ask, collect."""

import glob
import json
import os
import queue
import subprocess
import sys
import threading

import numpy as np

from .chrono_env import _conda_roots
from .stereo_worker import pack, unpack


class DepthWorker:
    """The process that runs the depth networks (stereo_worker.py). Requests are answered in the
    order they were made; a thread keeps reading the answers so that neither side waits on a
    full pipe. The process may be on another machine: its pipes then run through ssh. The link
    is then what takes the time, so the answers come back as 16 bit floats, and images and
    answers travel packed, without loss (see pack in stereo_worker.py)."""

    def __init__(self, command, remote=False):
        self.remote = remote
        self.proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        line = self.proc.stdout.readline()
        if not line:
            raise RuntimeError("the depth networks did not start. Try: %s --check" % " ".join(command))
        self.info = json.loads(line)
        self.info["where"] = command[-2] if remote else "this machine"
        self.pack = remote and bool(self.info.get("pack"))      # (a worker from before packing does not say so)
        self.replies = queue.Queue()
        self.sent, self.seconds = 0, 0.0
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        out = self.proc.stdout
        try:
            while True:
                line = out.readline()
                if not line:
                    break
                head = json.loads(line)
                size = head["h"] * head["w"]
                kind = np.dtype(head.get("dtype", "float32"))
                keep = np.float32 if kind.kind == "f" else kind          # (labels stay what they are)
                if "sizes" in head:
                    maps = [unpack(out.read(n), (head["h"], head["w"]), kind).astype(keep) for n in head["sizes"]]
                else:
                    maps = [np.frombuffer(out.read(kind.itemsize * size), kind).reshape(head["h"], head["w"]).astype(keep)
                            for _ in range(head["n"])]
                self.replies.put((head, maps))
        finally:
            self.replies.put((None, None))

    def submit(self, op, images):
        """Ask for the disparity of a pair ('stereo'), the relative inverse depth of each image
        ('mono') or the labels of each image ('scene'). Images are (rows, columns, 3) uint8 with
        the top row first."""
        self.sent += 1
        h, w = images[0].shape[:2]
        pipe = self.proc.stdin
        head = dict(id=self.sent, op=op, h=h, w=w, n=len(images), half=self.remote)
        data = [pack(img) if self.pack else np.ascontiguousarray(img).tobytes() for img in images]
        if self.pack:
            head["sizes"] = [len(b) for b in data]
        pipe.write((json.dumps(head) + "\n").encode())
        for b in data:
            pipe.write(b)
        pipe.flush()
        return self.sent

    def collect(self, ident):
        head, maps = self.replies.get()
        if head is None or head["id"] != ident:
            raise RuntimeError("the depth networks stopped")
        self.seconds += head["seconds"]
        return maps


def find_depth_python(explicit=None):
    """A Python that has what the depth networks need: the one named, this one, or a conda env."""
    need = ("torch", "timm", "transformers")

    def fits(py):          # looked up on disk, which is quick; importing PyTorch to ask takes seconds
        env = os.path.dirname(os.path.dirname(os.path.abspath(py)))
        return all(glob.glob(os.path.join(env, "lib", "python*", "site-packages", n, "__init__.py")) for n in need)

    explicit = explicit or os.environ.get("PARKING_DEPTH_PYTHON")
    if explicit:
        return explicit
    found = [sys.executable] if fits(sys.executable) else []
    for root in _conda_roots():
        found += [py for py in sorted(glob.glob(os.path.join(root, "envs", "*", "bin", "python"))) if fits(py)]
    return found[0] if found else None
