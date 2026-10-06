#!/usr/bin/env python3
# =============================================================================
# The networks of parking_sim.py
#
# Two depth networks, and one that labels what is in an image, in a process of their own:
#
#   stereo   IGEV++ (Xu et al., "IGEV++: Iterative Multi-range Geometry Encoding
#            Volumes for Stereo Matching", github.com/gangweiX/IGEV-plusplus,
#            MIT licence) on a rectified image pair. Returns the disparity of
#            the left image in pixels.
#   mono     Depth Anything V2 Small (Yang et al., Apache 2.0 licence), through
#            the transformers package, on single images. Returns relative
#            inverse depth: it has neither scale nor offset, parking_sim.py
#            fixes both with the ground it knows.
#   scene    Mask2Former with weights trained on Mapillary Vistas (scene_net.py),
#            on single images. Returns a label per pixel: painted marking,
#            kerb, the car's own body, or none of these.
#
# It is a separate process because the networks need PyTorch, which the Python
# that runs Chrono usually does not have, and because the two bring their own
# OpenMP runtimes, which cannot share a process on macOS.
#
# parking_sim.py starts it and talks to it over its stdin and stdout:
#
#   request    one line of JSON {"id": n, "op": "stereo", "mono" or "scene",
#              "h": rows, "w": columns, "n": images}, then the images, each rows x
#              columns x 3 bytes (RGB, top row first). A stereo request has two:
#              left, right
#   reply      one line of JSON {"id": n, "h": rows, "w": columns, "n": maps,
#              "seconds": t, "dtype": type}, then the maps, each rows x columns
#              float32, or float16 if the request said "half": true (for a slow
#              link). The labels of a scene request are uint8
#
# For a slow link the images and the maps can also travel packed (see pack below):
# the request then carries "sizes", the number of bytes of each packed image, and
# the reply does the same for its maps, which are float16. Nothing is lost.
#
# After loading the networks it prints one line {"ready": true, ...}. Everything
# else it has to say goes to stderr.
#
#   python parking/stereo_worker.py --check
#
# runs made-up images through the networks and prints the times, to test the set-up.
# =============================================================================

import argparse
import json
import os
import sys
import time
import types
import zlib

import numpy as np


def pack(a):
    """An image (uint8) or a map (float16) in fewer bytes, without loss: each value less the one
    to its left, then zlib. A camera image shrinks to 0.6 of its size."""
    v = np.ascontiguousarray(a).view(np.uint8 if a.dtype == np.uint8 else np.uint16)
    d = v.copy()
    d[:, 1:] -= v[:, :-1]
    return zlib.compress(d.tobytes(), 1)


def unpack(blob, shape, dtype):
    d = np.frombuffer(zlib.decompress(blob), np.uint8 if dtype == np.uint8 else np.uint16).reshape(shape)
    return np.cumsum(d, axis=1, dtype=d.dtype).view(dtype)


def load_model(repo, checkpoint, realtime, device):
    """The network of the IGEV++ repository with the weights of a checkpoint, ready for inference."""
    import torch
    import timm

    create = timm.create_model

    def create_bare(name, pretrained=False, **kw):
        # The backbone's weights are in the checkpoint, so nothing is fetched from the model hub.
        # timm 1.x folds the first activation into bn1; the repository was written for 0.5.
        model = create(name, pretrained=False, **kw)
        if not hasattr(model, "act1"):
            model.act1 = torch.nn.Identity()
        return model

    timm.create_model = create_bare
    sys.path.insert(0, repo)
    sys.path.insert(0, os.path.join(repo, "core_rt" if realtime else "core"))
    if realtime:
        from core_rt.rt_igev_stereo import IGEVStereo
        args = types.SimpleNamespace(mixed_precision=False, precision_dtype="float32", hidden_dim=96, corr_levels=2,
                                     corr_radius=4, n_downsample=2, n_gru_layers=1, max_disp=192)
    else:
        from core.igev_stereo import IGEVStereo
        args = types.SimpleNamespace(mixed_precision=False, precision_dtype="float32", hidden_dims=[128] * 3,
                                     corr_levels=2, corr_radius=4, n_downsample=2, n_gru_layers=3, max_disp=768,
                                     s_disp_range=48, m_disp_range=96, l_disp_range=192, s_disp_interval=1,
                                     m_disp_interval=2, l_disp_interval=4)
    model = IGEVStereo(args)
    state = torch.load(checkpoint, map_location="cpu", weights_only=True)
    state = {(k[7:] if k.startswith("module.") else k): v for k, v in state.items()}
    missing, unexpected = model.load_state_dict(state, strict=False)
    if missing or unexpected:
        raise RuntimeError("checkpoint %s does not fit the network: %d tensors missing, %d not used"
                           % (checkpoint, len(missing), len(unexpected)))
    return model.to(device).eval()


class Matcher:
    def __init__(self, repo, checkpoint, realtime, device, iters):
        import torch
        self.torch = torch
        if device == "auto":
            device = "cuda" if torch.cuda.is_available() else "mps" if torch.backends.mps.is_available() else "cpu"
        self.device, self.iters = torch.device(device), iters
        # Left to itself, cuDNN picks convolution routines that do not give the same answer twice:
        # on an RTX 5070 Ti the disparity of one and the same pair came out more than 0.1 pixels
        # different on 13 percent of the pixels nearer than 14 m. The repeatable routines are no
        # slower here, and with them a run can be repeated.
        torch.backends.cudnn.benchmark = False
        torch.backends.cudnn.deterministic = True
        self.model = load_model(repo, checkpoint, realtime, self.device)

    def disparity(self, left, right):
        """Disparity of the left image [pixels] for a rectified pair of (rows, columns, 3) uint8 images."""
        torch, F = self.torch, self.torch.nn.functional
        h, w = left.shape[:2]
        ph, pw = (-h) % 32, (-w) % 32                # the network wants sizes that divide by 32
        pad = [pw // 2, pw - pw // 2, ph // 2, ph - ph // 2]

        def tensor(img):
            t = torch.from_numpy(np.ascontiguousarray(img)).permute(2, 0, 1).float()[None]
            return F.pad(t, pad, mode="replicate").to(self.device)

        with torch.no_grad():
            out = self.model(tensor(left), tensor(right), iters=self.iters, test_mode=True)
        disp = out.float().cpu().numpy()[0, 0]
        return np.ascontiguousarray(disp[pad[2]:pad[2] + h, pad[0]:pad[0] + w], dtype=np.float32)


class MonoDepth:
    """Depth Anything V2: relative inverse depth of single images."""

    def __init__(self, name, device):
        import torch
        from transformers import AutoImageProcessor, AutoModelForDepthEstimation
        self.torch, self.device = torch, device
        self.processor = AutoImageProcessor.from_pretrained(name)
        self.model = AutoModelForDepthEstimation.from_pretrained(name).to(device).eval()

    def inverse_depth(self, images):
        """Relative inverse depth, (rows, columns) float32 per image, for (rows, columns, 3) uint8 images."""
        torch = self.torch
        h, w = images[0].shape[:2]
        inp = self.processor(images=[np.ascontiguousarray(im) for im in images], return_tensors="pt").to(self.device)
        with torch.no_grad():
            out = self.model(**inp).predicted_depth
            out = torch.nn.functional.interpolate(out[:, None], size=(h, w), mode="bicubic", align_corners=False)
        return [np.ascontiguousarray(m, dtype=np.float32) for m in out[:, 0].float().cpu().numpy()]


def read_exact(stream, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = stream.read(n - len(buf))
        if not chunk:
            raise EOFError
        buf += chunk
    return bytes(buf)


def serve(matcher, mono, scene, info):
    stdin, stdout = sys.stdin.buffer, sys.stdout.buffer
    stdout.write((json.dumps(dict(info, ready=True, pack=True)) + "\n").encode())
    stdout.flush()
    while True:
        line = stdin.readline()
        if not line:
            return
        req = json.loads(line)
        h, w, n = req["h"], req["w"], req.get("n", 2)
        sizes = req.get("sizes")
        if sizes:
            images = [unpack(read_exact(stdin, s), (h, w, 3), np.uint8) for s in sizes]
        else:
            images = [np.frombuffer(read_exact(stdin, h * w * 3), np.uint8).reshape(h, w, 3) for _ in range(n)]
        t0 = time.perf_counter()
        op = req.get("op", "stereo")
        if op == "scene":
            maps, kind = scene.labels(images), "uint8"
        else:
            maps = [matcher.disparity(*images)] if op == "stereo" else mono.inverse_depth(images)
            kind = "float16" if req.get("half") or sizes else "float32"
        reply = dict(id=req.get("id"), h=h, w=w, n=len(maps), seconds=round(time.perf_counter() - t0, 4), dtype=kind)
        data = [pack(m.astype(kind)) if sizes else m.astype(kind).tobytes() for m in maps]
        if sizes:
            reply["sizes"] = [len(b) for b in data]
        stdout.write((json.dumps(reply) + "\n").encode())
        for b in data:
            stdout.write(b)
        stdout.flush()


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="The networks of parking_sim.py: IGEV++, Depth Anything V2, and "
                                                 "Mask2Former for what is in an image")
    ap.add_argument("--repo", default=os.environ.get("IGEV_ROOT", os.path.join(os.path.dirname(here), "third_party",
                                                                          "IGEV-plusplus")),
                    help="checkout of github.com/gangweiX/IGEV-plusplus")
    ap.add_argument("--model", choices=("igev", "rt"), default="igev", help="IGEV++ or its real-time version")
    ap.add_argument("--weights", default=None, help="checkpoint file (default: the repository's pretrained_models)")
    ap.add_argument("--iters", type=int, default=8, help="disparity update iterations")
    ap.add_argument("--mono", default="depth-anything/Depth-Anything-V2-Small-hf",
                    help="monocular depth model on the Hugging Face hub or in a directory, 'none' to leave it out")
    ap.add_argument("--scene", default="none",
                    help="network that labels markings, kerbs and the car's own body: a Mask2Former model on the "
                         "Hugging Face hub or in a directory with the classes of Mapillary Vistas, 'none' to leave it "
                         "out, 'auto' for the one of scene_net.py if the device is a CUDA GPU")
    ap.add_argument("--device", default="auto", help="auto, cuda, mps or cpu")
    ap.add_argument("--check", action="store_true", help="run made-up images through the networks and exit")
    a = ap.parse_args()
    os.environ.setdefault("PYTORCH_ENABLE_MPS_FALLBACK", "1")
    os.environ.setdefault("TRANSFORMERS_VERBOSITY", "error")
    os.environ.setdefault("HF_HUB_DISABLE_PROGRESS_BARS", "1")
    import warnings
    warnings.filterwarnings("ignore")
    # The checkpoint trained for Middlebury (on a mix of data sets) is the most accurate on the
    # simulated cameras up to 15 m. The real-time network only comes with Scene Flow weights.
    weights = a.weights or os.path.join(a.repo, "pretrained_models",
                                        "igev_rt/sceneflow.pth" if a.model == "rt" else "igev_plusplus/middlebury.pth")
    for path, what in ((a.repo, "IGEV++ repository"), (weights, "weights")):
        if not os.path.exists(path):
            sys.exit("stereo worker: %s not found at %s (see docs/sensors.md)" % (what, path))
    stdout, sys.stdout = sys.stdout, sys.stderr      # whatever the libraries print must not end up in the replies
    matcher = Matcher(a.repo, weights, a.model == "rt", a.device, a.iters)
    mono = None if a.mono == "none" else MonoDepth(a.mono, matcher.device)
    scene = None
    if a.scene != "none":
        sys.path.insert(0, here)
        from scene_net import NAME, SceneNet
        if a.scene == "auto":          # 0.06 s per image on a desktop GPU, 0.4 s on Apple silicon
            a.scene = NAME if matcher.device.type == "cuda" else "none"
    if a.scene != "none":
        scene = SceneNet(a.scene, matcher.device)
    sys.stdout = stdout
    info = dict(model="RT-IGEV++" if a.model == "rt" else "IGEV++", weights=os.path.basename(weights),
                device=str(matcher.device), iters=a.iters, mono=None if mono is None else a.mono.split("/")[-1],
                scene=None if scene is None else a.scene.split("/")[-1])
    if a.check:
        rng = np.random.default_rng(0)
        base = rng.integers(0, 255, (600, 1000, 3), dtype=np.uint8)
        base = np.kron(base[::8, ::8], np.ones((8, 8, 1), np.uint8))[:600, :1000]      # blocky texture
        left, right = base[:, 20:980], base[:, 32:992]                               # the right view, 12 pixels over
        for k in range(3):
            t0 = time.perf_counter()
            disp = matcher.disparity(left, right)
            print("%s on %s, %d x %d: %.2f s, median disparity %.2f px (12 expected)" % (
                info["model"], info["device"], left.shape[1], left.shape[0], time.perf_counter() - t0,
                float(np.median(disp))))
        for k in range(3 if mono is not None else 0):
            t0 = time.perf_counter()
            out = mono.inverse_depth([left, right])
            print("%s, 2 images of %d x %d: %.2f s, output %s" % (info["mono"], left.shape[1], left.shape[0],
                                                              time.perf_counter() - t0, out[0].shape))
        for k in range(3 if scene is not None else 0):
            t0 = time.perf_counter()
            out = scene.labels([left])
            print("%s, 1 image of %d x %d: %.2f s, output %s" % (info["scene"], left.shape[1], left.shape[0],
                                                             time.perf_counter() - t0, out[0].shape))
        return
    try:
        serve(matcher, mono, scene, info)
    except (BrokenPipeError, EOFError):          # the simulation is gone
        os._exit(0)


if __name__ == "__main__":
    main()
