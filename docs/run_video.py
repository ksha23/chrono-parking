#!/usr/bin/env python3
"""A video of a run: the scene from above with what the car believes drawn on it, and what its
front camera sees.

    python docs/run_video.py OUT --type angled --cars both --seed 10
    python docs/run_video.py --encode OUT          the video from frames made on another machine

The arguments after OUT are those of parking_sim.py. The run is the one parking_sim.py makes with
them, headless. A camera 20 m above the car looks straight down and moves along with it. It is
no sensor of the car: it is there for the video. On its picture are drawn

    yellow            the painted lines the car has found
    green, red, grey  the stalls it makes of them: free, taken, not known yet
    white             the stall it has chosen, and the car as it means to stand in it
    cyan, orange      its plan, forwards and backwards, and where it believes it is
    red dots          cells of its map with an obstacle in them
    blue dots         kerb that the scene network has pointed out
    dotted green      the stalls that really are free (the car is not told)

Frames are written to OUT/frame_00000.jpg and so on, one per 0.1 s of the run, and if ffmpeg is
installed they are made into OUT.mp4 at twice the speed of the run. The frames are drawn in a
second process, with the Python of the networks: that one has an image library, and the Python
that has PyChrono usually has none. Needs the sensor rig (docs/sensors.md)."""

import glob
import json
import math
import os
import shutil
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

W, H = 1280, 800                     # of the picture from above
HEIGHT, HFOV = 20.0, math.radians(75.0)
FOCAL = 0.5 * W / math.tan(0.5 * HFOV)
SMALL = (480, 300)                   # the front camera's picture in the corner
SPEED = 2                            # the video runs this many times faster than the run


def record(out, sim_args):
    # (imported before the arguments are swapped: see docs/rate_check.py)
    from parking.agent import ParkingSim
    from parking.chrono_env import chrono, sens
    from parking.cli import parse_args
    from parking.geometry import ego_poly
    from parking.networks import find_depth_python
    from parking.sensors import SensorRig
    from parking.world import light_scene
    sys.argv = ["parking_sim.py", "--headless"] + sim_args
    args = parse_args()
    sim = ParkingSim(args)
    if not isinstance(sim.sensor, SensorRig):
        sys.exit("a video needs the sensor rig (--sensors camera or camera+lidar)")
    python = find_depth_python(args.depth_python)
    if python is None:
        sys.exit("the frames are drawn with the Python of the networks, and none was found (--depth-python)")
    os.makedirs(out, exist_ok=True)
    draw = subprocess.Popen([python, os.path.abspath(__file__), "--draw", out], stdin=subprocess.PIPE)

    # The camera above the car, in a second sensor manager: the car's own sensors, and the random
    # numbers they render with, stay as they are. It is mounted on the car's body and turned
    # back in every frame, so that it looks straight down with +x to the right whichever way
    # the car points. (A body of its own to hang it on would do that more simply, but one more
    # body in the system, even a fixed one that touches nothing, makes a different run: the
    # same scenario then parked where it had not.)
    world = sim.world
    body = world.car.GetChassisBody()
    manager = sens.ChSensorManager(world.system)
    exposure, _ = light_scene(manager.scene, args.sky)
    down = chrono.QuatFromAngleZ(0.5 * math.pi) * chrono.QuatFromAngleY(0.5 * math.pi)       # +x to the right, +y up
    above = sens.ChCameraSensor(body, 10.5, chrono.ChFramed(chrono.ChVector3d(0.0, 0.0, HEIGHT), down), W, H, HFOV, 1)
    above.PushFilter(sens.ChFilterRGBA8Access())
    above.SetLag(0.0)
    above.SetCollectionWindow(0.0)
    manager.AddSensor(above)
    gain = exposure ** (1.0 / 2.2)                 # (a renderer without an exposure setting: see SensorRig.lens)

    free = [np.round(q["corners"], 2).tolist() for q in sim.scn.stalls if not q["occupied"]]
    grid, last = sim.grid, -1.0
    skip = [i for i, a in enumerate(sim_args) if a in ("--depth-python", "--depth-host", "--depth-dir")]
    title = " ".join(a for i, a in enumerate(sim_args) if i not in skip and i - 1 not in skip) + "   (at %d times the speed)" % SPEED

    def frame(hold=1, result=None):
        x, y = sim.true_pose[:2]
        ref = body.GetFrameRefToAbs()
        above.SetOffsetPose(chrono.ChFramed(ref.TransformPointParentToLocal(chrono.ChVector3d(x, y, HEIGHT)),
                                            ref.GetRot().GetConjugate() * down))
        manager.Update()
        buf = above.GetMostRecentRGBA8Buffer()
        if not buf.HasData():
            return
        top = np.minimum(buf.GetRGBA8Data()[::-1, :, :3] * gain + 0.5, 255.0).astype(np.uint8)
        front = sim.sensor.left.get("image")
        front = np.zeros((2 * SMALL[1], 2 * SMALL[0], 3), np.uint8) if front is None else front[::-1]
        front = front.reshape(SMALL[1], 2, SMALL[0], 2, 3).mean(axis=(1, 3)).astype(np.uint8)
        near = lambda p: p[(np.abs(p[:, 0] - x) < 19.0) & (np.abs(p[:, 1] - y) < 12.0)]
        iy, ix = np.nonzero(grid.kerb >= 0.35)
        kerb = np.stack([grid.x0 + (ix + 0.5) * grid.RES, grid.y0 + (iy + 0.5) * grid.RES], axis=1)
        target = sim.target
        head = dict(
            t=sim.time, state=sim.state, v=float(sim.speed), text=getattr(sim, "message", ""), title=title, at=[x, y],
            hold=hold, result=result, free=free,
            lines=[np.round(np.concatenate(k.ends()), 2).tolist() for k in sim.lines.markers()],
            slots=[dict(c=np.round(s.corners, 2).tolist(), s=s.status, t=s is target) for s in sim.slots],
            goal=None if target is None else np.round(ego_poly(target.goal(sim.nose_in, sim.park_dir)), 2).tolist(),
            path=[dict(d=seg.dir, p=np.round(np.stack([seg.x, seg.y], axis=1)[::3], 2).tolist()) for seg in sim.path[sim.seg_i:]],
            car=np.round(ego_poly(sim.pose), 2).tolist(),
            hits=np.round(near(grid.occupied_points()), 2).tolist(), kerb=np.round(near(kerb), 2).tolist())
        draw.stdin.write((json.dumps(head) + "\n").encode())
        draw.stdin.write(top.tobytes())
        draw.stdin.write(front.tobytes())
        draw.stdin.flush()

    while sim.result is None and sim.time < 180.0:
        sim.advance(100)
        if sim.time > last + 0.05:             # (the clock stands still while the planner works)
            last = sim.time
            frame()
    res = sim.result or {}
    words = "parked" if res.get("ok") else "NOT PARKED"
    if "min_clearance" in res:
        words += ", nearest it came to anything: %.2f m" % res["min_clearance"]
    if "depth" in res:
        words += ", off the stall's middle by %+.2f m along it and %+.2f m across" % (res["depth"], res["lateral"])
    frame(hold=30 * SPEED, result=words)
    draw.stdin.close()
    draw.wait()
    encode(out)
    sys.stdout.flush()
    os._exit(0)             # (see the note on leaving in parking/cli.py)


def encode(out):
    frames = len(glob.glob(os.path.join(out, "frame_*.jpg")))
    if not frames:
        sys.exit("no frames in %s" % out)
    if shutil.which("ffmpeg") is None:
        print("%d frames in %s. No ffmpeg here: copy them to a machine that has it and run\n"
              "    python docs/run_video.py --encode %s" % (frames, out, out), flush=True)
        return
    video = out.rstrip("/") + ".mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-framerate", str(10 * SPEED), "-i", os.path.join(out, "frame_%05d.jpg"),
                    "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "26", video], check=True)
    print("%s: %d frames, %.0f s" % (video, frames, frames / (10.0 * SPEED)), flush=True)


def draw(out):
    """Read frames from standard input, draw on them, write them as pictures."""
    from PIL import Image, ImageDraw, ImageFont

    def font(size):
        try:
            return ImageFont.load_default(size=size)
        except TypeError:                       # (an older Pillow: one size only)
            return ImageFont.load_default()

    big, small = font(20), font(15)
    colour = dict(free=(60, 220, 90), occupied=(235, 70, 60), unknown=(170, 170, 170))
    legend = (((255, 220, 40), "lines found"), ((60, 220, 90), "stall: free"), ((235, 70, 60), "stall: taken"),
              ((255, 255, 255), "chosen stall, and the car in it"), ((0, 220, 255), "plan forwards, believed position"),
              ((255, 150, 30), "plan backwards"), ((255, 60, 60), "obstacle in the map"), ((70, 130, 255), "kerb, by the network"),
              ((150, 255, 150), "really free (dotted)"))
    stream, count = sys.stdin.buffer, 0

    def read(n):
        data = bytearray()
        while len(data) < n:
            chunk = stream.read(n - len(data))
            if not chunk:
                raise EOFError
            data += chunk
        return bytes(data)

    while True:
        line = stream.readline()
        if not line:
            break
        f = json.loads(line)
        img = Image.frombytes("RGB", (W, H), read(W * H * 3))
        front = Image.frombytes("RGB", SMALL, read(SMALL[0] * SMALL[1] * 3))
        d = ImageDraw.Draw(img)
        cx, cy = f["at"]
        px = lambda p: (0.5 * W + FOCAL * (p[0] - cx) / HEIGHT, 0.5 * H - FOCAL * (p[1] - cy) / HEIGHT)
        ring = lambda pts, fill, width: d.line([px(p) for p in pts] + [px(pts[0])], fill=fill, width=width)
        for p in f["hits"]:
            u, v = px(p)
            d.rectangle([u - 1, v - 1, u + 1, v + 1], fill=(255, 60, 60))
        for p in f["kerb"]:
            u, v = px(p)
            d.rectangle([u - 1, v - 1, u + 1, v + 1], fill=(70, 130, 255))
        for c in f["free"]:                     # dotted: every other 0.25 m of the outline
            for a, b in zip(c, c[1:] + c[:1]):
                n = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.25))
                for k in range(0, n, 2):
                    q0, q1 = ((a[0] + (b[0] - a[0]) * j / n, a[1] + (b[1] - a[1]) * j / n) for j in (k, k + 1))
                    d.line([px(q0), px(q1)], fill=(150, 255, 150), width=2)
        for x1, y1, x2, y2 in f["lines"]:
            d.line([px((x1, y1)), px((x2, y2))], fill=(255, 220, 40), width=3)
        for s in f["slots"]:
            if not s["t"]:
                ring(s["c"], colour[s["s"]], 2)
        for seg in f["path"]:
            if len(seg["p"]) > 1:
                d.line([px(p) for p in seg["p"]], fill=(0, 220, 255) if seg["d"] > 0 else (255, 150, 30), width=3)
        for s in f["slots"]:
            if s["t"]:
                ring(s["c"], (255, 255, 255), 4)
        if f["goal"]:
            ring(f["goal"], (255, 255, 255), 2)
        ring(f["car"], (0, 220, 255), 2)
        # the front camera, the legend, and what is going on
        img.paste(front, (W - SMALL[0] - 12, 12))
        d.rectangle([W - SMALL[0] - 13, 11, W - 12, 12 + SMALL[1]], outline=(255, 255, 255))
        d.text((W - SMALL[0] - 6, 16), "front camera", font=small, fill=(255, 255, 255))
        d.rectangle([0, 0, 12 + d.textlength(f["title"], font=big) + 12, 30], fill=(0, 0, 0))
        d.text((10, 4), f["title"], font=big, fill=(255, 255, 255))
        d.rectangle([8, H - 60 - 20 * len(legend), 290, H - 52], fill=(0, 0, 0))
        for k, (rgb, what) in enumerate(legend):
            d.rectangle([14, H - 54 - 20 * (len(legend) - k), 26, H - 62 - 20 * (len(legend) - k - 1)], fill=rgb)
            d.text((34, H - 58 - 20 * (len(legend) - k)), what, font=small, fill=(255, 255, 255))
        d.rectangle([0, H - 44, W, H], fill=(0, 0, 0))
        words = f["result"] or "t = %5.1f s   %-7s %4.1f m/s   %s" % (f["t"], f["state"], f["v"], f["text"])
        d.text((10, H - 36), words[:120], font=big, fill=(255, 235, 120) if f["result"] else (255, 255, 255))
        for _ in range(f["hold"]):
            img.save(os.path.join(out, "frame_%05d.jpg" % count), quality=88)
            count += 1


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "--draw":
        draw(sys.argv[2])
    elif len(sys.argv) == 3 and sys.argv[1] == "--encode":
        encode(sys.argv[2])
    elif len(sys.argv) >= 2 and not sys.argv[1].startswith("-"):
        record(sys.argv[1], sys.argv[2:])
    else:
        sys.exit(__doc__)
