import os
from typing import Callable

import cv2
import numpy as np
import torch
from dora import DoraStatus

DEVICE = os.environ.get("PYTORCH_DEVICE") or "cpu"


class Operator:
    """
    Infering object from images
    """

    def __init__(self):
        if os.environ.get("DEPTH_PATH") is None:
            # With internet
            self.model = torch.hub.load(
                "intel-isl/MiDaS",
                "MiDaS_small",
            )
            self.midas_transform = torch.hub.load("intel-isl/MiDaS", "transforms").small_transform

        else:
            # Without internet
            self.model = torch.hub.load(
                os.environ.get("YOLOV5_PATH"),
                "custom",
                path=os.environ.get("YOLOV5_WEIGHT_PATH"),
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
        if dora_input["id"] == "check":
            send_output("ready", b"")
            return DoraStatus.CONTINUE

        else:
            frame = cv2.imdecode(
                np.frombuffer(
                    dora_input["data"],
                    dtype="uint8",
                ),
                -1,
            )
            frame = frame[:, :, :3]
            input_batch = self.midas_transform(frame).to(torch.device(DEVICE))
            with torch.no_grad():
                prediction = self.model(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=frame.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
                arrays = prediction.cpu().numpy().tobytes()

            send_output("depth_frame", arrays, dora_input["metadata"])
            return DoraStatus.CONTINUE
