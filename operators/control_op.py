import threading
from pickle import dumps, loads

import numpy as np
from carla import Client, VehicleControl, command

CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"
client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
client.set_timeout(30.0)

mutex = threading.Lock()
from typing import Callable


class Operator:
    """
    Execute any `control` it r:ecieve given a `vehicle_id`
    """

    def __init__(self):
        self.vehicle_id = None

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ):
        """Handle input.
        Args:
            input_id (str): Id of the input declared in the yaml configuration
            value (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """

        if self.vehicle_id is None and "vehicle_id" != input_id:
            return {}
        elif self.vehicle_id is None and "vehicle_id" == input_id:
            #   global mutex
            #   mutex.acquire()
            self.vehicle_id = int.from_bytes(value, "big")
        #   mutex.release()
        elif self.vehicle_id is not None and "vehicle_id" == input_id:
            return {}

        if "control" != input_id:
            return {}

        control = np.frombuffer(value)

        vec_control = VehicleControl(
            throttle=control[0],
            steer=control[1],
            brake=control[2],
            hand_brake=False,
            reverse=False,
        )

        client.apply_batch_sync(
            [command.ApplyVehicleControl(self.vehicle_id, vec_control)]
        )
