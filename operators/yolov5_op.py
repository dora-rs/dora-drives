from typing import Callable

import cv2
import numpy as np
import torch

from dora_utils import DoraStatus


class Operator:
    """
    Infering object from images
    """

    def __init__(self):
        self.model = torch.hub.load("ultralytics/yolov5", "yolov5n")

    def on_input(
        self,
        _input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        """Handle image
        Args:
            input_id (str): Id of the input declared in the yaml configuration
            value (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """

        frame = cv2.imdecode(
            np.frombuffer(
                value,
                dtype="uint8",
            ),
            -1,
        )
        frame = frame[:, :, :3]

        results = self.model(frame)  # includes NMS
        arrays = np.array(results.xyxy[0].cpu())[
            :, [0, 2, 1, 3, 4, 5]
        ]  # xyxy -> xxyy
        arrays[:, 4] *= 100
        arrays = arrays.astype(np.int32)
        arrays = arrays.tobytes()
        send_output("bbox", arrays)
        return DoraStatus.CONTINUE
