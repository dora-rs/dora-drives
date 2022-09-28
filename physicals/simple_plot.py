import os
from enum import Enum
from typing import Callable

import cv2
import numpy as np


class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1


class Operator:
    """
    Plot image and bounding box
    """

    def __init__(self):
        self.image = []

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        """
        Put image on a cv2 window.
        Args:
            input_id (str): Id of the input declared in the yaml configuration
            value (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """
        if input_id == "image":
            frame = np.frombuffer(value, dtype="uint8")
            frame = cv2.imdecode(frame, -1)
            self.image = frame
            cv2.imshow("frame", self.image)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                return DoraStatus.STOP

        return DoraStatus.CONTINUE

    def drop_operator(self):
        cv2.destroyAllWindows()
