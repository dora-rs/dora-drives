from typing import Callable

import cv2
import numpy as np
from enum import Enum


class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.video_capture = cv2.VideoCapture(0)
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        ret, frame = self.video_capture.read()
        send_output("image", cv2.imencode(".jpg", frame)[1].tobytes())
        return DoraStatus.CONTINUE

    def drop_operator(self):
        self.video_capture.release()
