#!/usr/bin/env python3
# =============================================================================
# Depth networks for parking_sim.py
#
# Two networks, in a process of their own:
#
#   stereo   IGEV++ (Xu et al., "IGEV++: Iterative Multi-range Geometry Encoding
#            Volumes for Stereo Matching", github.com/gangweiX/IGEV-plusplus,
#            MIT licence) on a rectified image pair. Returns the disparity of
#            the left image in pixels.
#   mono     Depth Anything V2 Small (Yang et al., Apache 2.0 licence), through
#            the transformers package, on single images. Returns relative
#            inverse depth: it has neither scale nor offset, parking_sim.py
#            fixes both with the ground it knows.
#
# It is a separate process because the networks need PyTorch, which the Python
# that runs Chrono usually does not have, and because the two bring their own
# OpenMP runtimes, which cannot share a process on macOS.
#
# parking_sim.py starts it and talks to it over its stdin and stdout:
#
#   request    one line of JSON {"id": n, "op": "stereo" or "mono", "h": rows,
#              "w": columns, "n": images}, then the images, each rows x columns
#              x 3 bytes (RGB, top row first). A stereo request has two: left, right
#   reply      one line of JSON {"id": n, "h": rows, "w": columns, "n": maps,
#              "seconds": t}, then the maps, each rows x columns float32
#
# After loading the networks it prints one line {"ready": true, ...}. Everything
# else it has to say goes to stderr.
#
#   python stereo_worker.py --check
#
# runs made-up images through the networks and prints the times, to test the set-up.
# =============================================================================

import argparse
import json
import os
import sys
import time
import types

import numpy as np


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


def serve(matcher, mono, info):
    stdin, stdout = sys.stdin.buffer, sys.stdout.buffer
    stdout.write((json.dumps(dict(info, ready=True)) + "\n").encode())
    stdout.flush()
    while True:
        line = stdin.readline()
        if not line:
            return
        req = json.loads(line)
        h, w, n = req["h"], req["w"], req.get("n", 2)
        images = [np.frombuffer(read_exact(stdin, h * w * 3), np.uint8).reshape(h, w, 3) for _ in range(n)]
        t0 = time.perf_counter()
        maps = [matcher.disparity(*images)] if req.get("op", "stereo") == "stereo" else mono.inverse_depth(images)
        reply = dict(id=req.get("id"), h=h, w=w, n=len(maps), seconds=round(time.perf_counter() - t0, 4))
        stdout.write((json.dumps(reply) + "\n").encode())
        for m in maps:
            stdout.write(m.tobytes())
        stdout.flush()


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="Depth networks for parking_sim.py: IGEV++ and Depth Anything V2")
    ap.add_argument("--repo", default=os.environ.get("IGEV_ROOT", os.path.join(here, "third_party", "IGEV-plusplus")),
                    help="checkout of github.com/gangweiX/IGEV-plusplus")
    ap.add_argument("--model", choices=("igev", "rt"), default="igev", help="IGEV++ or its real-time version")
    ap.add_argument("--weights", default=None, help="checkpoint file (default: the repository's pretrained_models)")
    ap.add_argument("--iters", type=int, default=8, help="disparity update iterations")
    ap.add_argument("--mono", default="depth-anything/Depth-Anything-V2-Small-hf",
                    help="monocular depth model on the Hugging Face hub or in a directory, 'none' to leave it out")
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
    sys.stdout = stdout
    info = dict(model="RT-IGEV++" if a.model == "rt" else "IGEV++", weights=os.path.basename(weights),
                device=str(matcher.device), iters=a.iters, mono=None if mono is None else a.mono.split("/")[-1])
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
        return
    try:
        serve(matcher, mono, info)
    except (BrokenPipeError, EOFError):          # the simulation is gone
        os._exit(0)


if __name__ == "__main__":
    main()
