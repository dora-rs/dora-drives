import threading
import time
from typing import Callable

import numpy as np
from dora_utils import DoraStatus
from dora_world import World
from hybrid_astar_planner.HybridAStar.hybrid_astar_wrapper import \
    apply_hybrid_astar

from carla import Client
from hd_map import HDMap

mutex = threading.Lock()


# Planning general
TARGET_SPEED = 10.0
NUM_WAYPOINTS_AHEAD = 30
GOAL_LOCATION = [234, 59, 39]
CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"

world = World()
world._goal_location = GOAL_LOCATION
client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
client.set_timeout(30.0)  # seconds
carla_world = client.get_world()
hd_map = HDMap(carla_world.get_map())



class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.position = []
        self.world = world
        self.hd_map = hd_map

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ):

        if input_id == "position":
            self.position = np.frombuffer(value)

        self.world.update(
            time.time(), self.position, self.obstacles, [], hd_map
        )

        (waypoints, target_speeds) = self.planner.run(
            time.time()
        )  # , open_drive)
        self.planner._world.waypoints = waypoints

        waypoints_array = np.concatenate(
            [waypoints.T, target_speeds.reshape(1, -1)]
        )

        send_output(
            "waypoints",
            waypoints_array.tobytes(),
        )
        
        return DoraStatus.CONTINUE
