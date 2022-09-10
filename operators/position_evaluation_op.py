import time
from typing import Callable

import numpy as np

from dora_utils import DoraStatus


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.previous_position = []
        self.true_relative_position = []

    def on_input(
        self,
        input_id: str,
        value: bytes,
        _send_output: Callable[[str, bytes], None],
    ):

        if "position" == input_id:
            position = np.frombuffer(value)

            if len(self.previous_position) != 0:
                self.true_relative_position = position - self.previous_position
                print(f"True Relative position: {self.true_relative_position}")
            self.previous_position = position
            return DoraStatus.CONTINUE

        if "relative_position" == input_id:
            relative_position = np.frombuffer(value, dtype=np.float32)

            print(f"Computed Relative position: {relative_position}")

            print(
                f"error: {relative_position - self.true_relative_position[:6]}"
            )
            return DoraStatus.CONTINUE

        return DoraStatus.CONTINUE
