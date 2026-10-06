"""What is in a camera image, by a network: painted markings, kerbs, the car's own body.

Mask2Former (Cheng et al., "Masked-attention Mask Transformer for Universal Image Segmentation")
with the weights its authors trained on Mapillary Vistas, 25 000 street photographs from all over
the world labelled in 65 classes, through the transformers package. Nothing was trained on the
simulated cameras. Mapillary Vistas is licensed for non-commercial use (CC BY-NC-SA), and so are
weights made from it.

The network runs in the process of the depth networks (stereo_worker.py). This module holds no
PyTorch at import: the sensor rig reads the label codes from it."""

import numpy as np

NAME = "facebook/mask2former-swin-large-mapillary-vistas-semantic"

# The classes that the parking simulation reads. Everything else is 0.
MARKING, KERB, OWN = 1, 2, 3


class SceneNet:
    """Labels for (rows, columns, 3) uint8 images with the top row first."""

    def __init__(self, name, device):
        import torch
        from transformers import AutoImageProcessor, Mask2FormerForUniversalSegmentation
        self.torch, self.device = torch, device
        # (the images go in at their own size: lines a few pixels wide do not survive a resize)
        self.processor = AutoImageProcessor.from_pretrained(name, do_resize=False)
        self.model = Mask2FormerForUniversalSegmentation.from_pretrained(name).to(device).eval()
        names = self.model.config.id2label
        self.code = np.zeros(max(names) + 1, dtype=np.uint8)
        for k, v in names.items():
            self.code[k] = MARKING if "marking" in v.lower() else KERB if v == "Curb" else OWN if v == "Ego Vehicle" else 0
        if not {MARKING, KERB, OWN} <= set(self.code.tolist()):
            raise RuntimeError("%s has no classes for lane markings, kerbs and the car itself" % name)

    def labels(self, images):
        """One (rows, columns) uint8 map of the codes above per image. The images of one call
        must be of one size."""
        torch = self.torch
        h, w = images[0].shape[:2]
        inp = self.processor(images=[np.ascontiguousarray(im) for im in images], return_tensors="pt").to(self.device)
        # (half precision where the device has it: the same labels in half the time)
        with torch.no_grad(), torch.autocast("cuda", dtype=torch.float16, enabled=self.device.type == "cuda"):
            out = self.model(**inp)
        maps = self.processor.post_process_semantic_segmentation(out, target_sizes=[(h, w)] * len(images))
        return [self.code[m.cpu().numpy()] for m in maps]
