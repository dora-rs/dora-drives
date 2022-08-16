import threading
import time
from typing import Callable

import numpy as np
from carla import Client
from hybrid_astar_planner.HybridAStar.hybrid_astar_wrapper import (
    apply_hybrid_astar,
)

from dora_utils import DoraStatus
from dora_world import World
from hd_map import HDMap

mutex = threading.Lock()


# Hybrid ASTAR
STEP_SIZE_HYBRID_ASTAR = 3.0
MAX_ITERATIONS_HYBRID_ASTAR = 2000
COMPLETION_THRESHOLD = 1.0
ANGLE_COMPLETION_THRESHOLD = 100
RAD_STEP = 4.0
RAD_UPPER_RANGE = 4.0
RAD_LOWER_RANGE = 4.0
OBSTACLE_CLEARANCE_HYBRID_ASTAR = 1.0
LANE_WIDTH_HYBRID_ASTAR = 6.0
RADIUS = 6.0
CAR_LENGTH = 4.0
CAR_WIDTH = 1.8


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


class HybridAStarPlanner:
    """Wrapper around the Hybrid A* planner.

    Note:
        Details can be found at `Hybrid A* Planner`_.

    Args:
        world: (:py:class:`~pylot.planning.world.World`): A reference to the
            planning world.
        (absl.: Object to be used to access absl

    .. _Hybrid A* Planner:
       https://github.com/erdos-project/hybrid_astar_planner
    """

    def __init__(self, world):
        self._world = world
        self._hyperparameters = {
            "step_size": STEP_SIZE_HYBRID_ASTAR,
            "max_iterations": MAX_ITERATIONS_HYBRID_ASTAR,
            "completion_threshold": COMPLETION_THRESHOLD,
            "angle_completion_threshold": ANGLE_COMPLETION_THRESHOLD,
            "rad_step": RAD_STEP,
            "rad_upper_range": RAD_UPPER_RANGE,
            "rad_lower_range": RAD_LOWER_RANGE,
            "obstacle_clearance": OBSTACLE_CLEARANCE_HYBRID_ASTAR,
            "lane_width": LANE_WIDTH_HYBRID_ASTAR,
            "radius": RADIUS,
            "car_length": CAR_LENGTH,
            "car_width": CAR_WIDTH,
        }

    def run(self, _timestamp, _ttd=None):
        """Runs the planner.

        Note:
            The planner assumes that the world is up-to-date.

        Returns:
            :py:class:`~pylot.planning.waypoints.Waypoints`: Waypoints of the
            planned trajectory.
        """

        obstacle_list = self._world.get_obstacle_list()

        if len(obstacle_list) == 0:
            # Do not use Hybrid A* if there are no obstacles.
            return self._world.follow_waypoints(TARGET_SPEED)

        # Hybrid a* does not take into account the driveable region.
        # It constructs search space as a top down, minimum bounding
        # rectangle with padding in each dimension.

        initial_conditions = self._compute_initial_conditions(obstacle_list)
        path_x, path_y, _, success = apply_hybrid_astar(
            initial_conditions, self._hyperparameters
        )

        if not success:
            return self._world.follow_waypoints(0)

        speeds = np.array([TARGET_SPEED] * len(path_x))

        output_wps = np.array([path_x, path_y]).T

        return output_wps, speeds

    def _compute_initial_conditions(self, obstacles):
        [x, y, _, _, yaw, _, _] = self._world.position
        start = np.array(
            [
                x,
                y,
                np.deg2rad(yaw),
            ]
        )
        end_index = min(
            NUM_WAYPOINTS_AHEAD,
            len(self._world.waypoints) - 1,
        )

        if end_index < 0:
            # If no more waypoints left. Then our location is our end wp.
            end = np.array(
                [
                    x,
                    y,
                    np.deg2rad(yaw),
                ]
            )

        else:
            [end_x, end_y] = self._world.waypoints[end_index]
            end = np.array(
                [
                    end_x,
                    end_y,
                    np.deg2rad(yaw),
                ]
            )

        initial_conditions = {
            "start": start,
            "end": end,
            "obs": obstacles,
        }
        return initial_conditions


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.obstacles = []
        self.position = []
        self.planner = HybridAStarPlanner(world)

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ):

        if input_id == "position":
            self.position = np.frombuffer(value)

        if "obstacles" == input_id:
            obstacles = value.split(b"\n")
            obstacles_prediction = []
            for obstacle in obstacles:
                if len(obstacle) % 12 == 0:
                    obstacle_buffer = np.frombuffer(obstacle, dtype="float32")

                    obstacle_position = np.reshape(obstacle_buffer, (-1, 3))
                    obstacles_prediction.append(obstacle_position)
            self.obstacles = obstacles_prediction
            return DoraStatus.CONTINUE

        self.planner._world.update(
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
