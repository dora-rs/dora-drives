import os
from typing import Callable

import cv2
import numpy as np
import torch
from dora import DoraStatus

DEVICE = os.environ.get("PYTORCH_DEVICE") or "cpu"
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
DEVICE = "cuda"
YOLOV5_PATH = "/home/dora/workspace/simulate/team_code/dependencies/yolov5"
YOLOV5_WEIGHT_PATH = (
    "/home/dora/workspace/simulate/team_code/dependencies/yolov5/yolov5n.pt"
)


class Operator:
    """
    Send `bbox` found by YOLOv5 on given `image`
    """

    def __init__(self):
        if YOLOV5_PATH is None:
            # With internet
            self.model = torch.hub.load(
                "ultralytics/yolov5",
                "yolov5n",
            )
        else:
            # Without internet
            self.model = torch.hub.load(
                YOLOV5_PATH,
                "custom",
                path=YOLOV5_WEIGHT_PATH,
                source="local",
            )

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
            frame = np.frombuffer(
                dora_input["data"],
                dtype="uint8",
            ).reshape((IMAGE_HEIGHT, IMAGE_WIDTH, 4))
            frame = frame[:, :, :3]

            results = self.model(frame)  # includes NMS
            arrays = np.array(results.xyxy[0].cpu())[
                :, [0, 2, 1, 3, 4, 5]
            ]  # xyxy -> xxyy
            arrays[:, 4] *= 100
            arrays = arrays.astype(np.int32)
            arrays = arrays.tobytes()
            send_output("bbox", arrays, dora_input["metadata"])
            return DoraStatus.CONTINUE
