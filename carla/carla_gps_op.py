import threading
from typing import Callable

import numpy as np

from _dora_utils import DoraStatus, closest_vertex
from _hd_map import HDMap
from carla import Client

mutex = threading.Lock()


# Planning general
TARGET_SPEED = 10.0
NUM_WAYPOINTS_AHEAD = 30
GOAL_LOCATION = [234, 59, 39]
CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self._goal_location = GOAL_LOCATION
        client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
        client.set_timeout(30.0)  # seconds
        carla_world = client.get_world()
        hd_map = HDMap(carla_world.get_map())
        self.position = []
        self.hd_map = hd_map
        self.waypoints = []
        self.target_speeds = []

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):

        self.position = np.frombuffer(dora_input["data"])

        if len(self.waypoints) != 0:
            (index, _) = closest_vertex(
                self.waypoints,
                np.array([self.position[:2]]),
            )

            self.waypoints = self.waypoints[index : index + NUM_WAYPOINTS_AHEAD]
            self.target_speeds = self.target_speeds[
                index : index + NUM_WAYPOINTS_AHEAD
            ]

        if len(self.waypoints) < NUM_WAYPOINTS_AHEAD / 2:

            waypoints = self.hd_map.compute_waypoints(
                self.position[:3], self._goal_location
            )[:NUM_WAYPOINTS_AHEAD]
            self.waypoints = waypoints
            self.target_speeds = np.array([0.0] * len(waypoints))

        waypoints_array = np.concatenate(
            [self.waypoints.T, self.target_speeds.reshape(1, -1)]
        )

        send_output(
            "gps_waypoints", waypoints_array.tobytes(), dora_input["metadata"]
        )  # World coordinate

        return DoraStatus.CONTINUE
