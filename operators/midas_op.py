import os
from typing import Callable

import cv2
import numpy as np
import torch
from dora import DoraStatus

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
DEVICE = os.environ.get("PYTORCH_DEVICE") or "cpu"
MIDAS_PATH = os.environ.get("MIDAS_PATH")
MIDAS_WEIGHT_PATH = os.environ.get("MIDAS_WEIGHT_PATH")
MODEL_TYPE = os.environ.get("MODEL_TYPE")
MODEL_NAME = os.environ.get("MODEL_NAME")


# example source: https://pytorch.org/hub/intelisl_midas_v2/
class Operator:
    def __init__(self):
        if MIDAS_PATH is None:
            # With internet
            self.model = torch.hub.load(
                "intel-isl/MiDaS",
                MODEL_TYPE,
            )
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        else:
            # Without internet
            self.model = torch.hub.load(
                repo_or_dir=MIDAS_PATH,
                model=MODEL_NAME,
                weights=MIDAS_WEIGHT_PATH,
                source="local",
            )
            midas_transforms = torch.hub.load(
                repo_or_dir=MIDAS_PATH, model="transforms", source="local"
            )
        if MODEL_TYPE == "DPT_Large" or MODEL_TYPE == "DPT_Hybrid":
            self.transform = midas_transforms.dpt_transform
        else:
            self.transform = midas_transforms.small_transform
        self.model.to(torch.device(DEVICE))
        self.model.eval()

    def on_event(
            self,
            dora_event: dict,
            send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
            self,
            dora_input: dict,
            send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        """Handle image
        Args:
            dora_input["id"](str): Id of the input declared in the yaml configuration
            dora_input["data"] (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """
        if dora_input["id"] == "image":
            # Convert bytes to numpy array
            frame = np.frombuffer(
                dora_input["data"],
                np.uint8,
            ).reshape((IMAGE_HEIGHT, IMAGE_WIDTH, 4))

            with torch.no_grad():
                image = frame[:, :, :3]
                img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                input_batch = self.transform(img).to(DEVICE)
                prediction = self.model(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=img.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
                depth_output = prediction.cpu().numpy()
                depth_min = depth_output.min()
                depth_max = depth_output.max()
                normalized_depth = 255 * (depth_output - depth_min) / (depth_max - depth_min)
                normalized_depth *= 3
                depth_frame = np.repeat(np.expand_dims(normalized_depth, 2), 3, axis=2) / 3
                depth_frame = cv2.applyColorMap(np.uint8(depth_frame), cv2.COLORMAP_INFERNO)
                height, width = depth_frame.shape[:2]
                depth_frame_4 = np.dstack([depth_frame, np.ones((height, width), dtype="uint8") * 255])

                send_output("depth_frame", depth_frame_4.tobytes(), dora_input["metadata"])
        return DoraStatus.CONTINUE
