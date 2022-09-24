import threading
import time
from typing import Callable

import numpy as np
from hybrid_astar_planner.HybridAStar.hybrid_astar_wrapper import (
    apply_hybrid_astar,
)

from dora_utils import DoraStatus, closest_vertex, pairwise_distances

mutex = threading.Lock()


# Hybrid ASTAR
STEP_SIZE_HYBRID_ASTAR = 3.0
MAX_ITERATIONS_HYBRID_ASTAR = 2000
COMPLETION_THRESHOLD = 1.0
ANGLE_COMPLETION_THRESHOLD = 100
RAD_STEP = 4.0
RAD_UPPER_RANGE = 4.0
RAD_LOWER_RANGE = 4.0
OBSTACLE_CLEARANCE_HYBRID_ASTAR = 0.5
LANE_WIDTH_HYBRID_ASTAR = 6.0
RADIUS = 6.0
CAR_LENGTH = 4.0
CAR_WIDTH = 1.8

OBSTACLE_DISTANCE_WAYPOINTS_THRESHOLD = 10
OBSTACLE_RADIUS = 1.0

# Planning general
TARGET_SPEED = 10.0
NUM_WAYPOINTS_AHEAD = 30
GOAL_LOCATION = [234, 59, 39]


def get_obstacle_list(obstacle_predictions, waypoints):
    if len(obstacle_predictions) == 0 or len(waypoints) == 0:
        return np.empty((0, 4))
    obstacle_list = []

    distances = pairwise_distances(waypoints, obstacle_predictions[:, :2]).min(
        0
    )
    for distance, prediction in zip(distances, obstacle_predictions):
        # Use all prediction times as potential obstacles.
        if distance < OBSTACLE_DISTANCE_WAYPOINTS_THRESHOLD:
            [x, y, _, _confidence, _label] = prediction
            obstacle_size = np.array(
                [
                    x - OBSTACLE_RADIUS,
                    y - OBSTACLE_RADIUS,
                    x + OBSTACLE_RADIUS,
                    y + OBSTACLE_RADIUS,
                ]
            )

            # Remove traffic light. TODO: Take into account traffic light.
            if _label != 9:
                obstacle_list.append(obstacle_size)

    if len(obstacle_list) == 0:
        return np.empty((0, 4))
    return np.array(obstacle_list)


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.obstacles = []
        self.position = []
        self.gps_waypoints = []
        self.waypoints = []
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

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ):

        if input_id == "gps_waypoints":

            waypoints = np.frombuffer(value)
            waypoints = waypoints.reshape((3, -1))
            self.gps_waypoints = waypoints[0:2].T
            if len(self.waypoints) == 0:
                self.waypoints = self.gps_waypoints
            return DoraStatus.CONTINUE

        if input_id == "obstacles":
            obstacles = np.frombuffer(value, dtype="float32").reshape((-1, 5))

            self.obstacles = obstacles
            return DoraStatus.CONTINUE

        if input_id == "position":
            self.position = np.frombuffer(value)

        if len(self.gps_waypoints) != 0:
            (waypoints, target_speeds) = self.run(time.time())  # , open_drive)
            self.waypoints = waypoints

            waypoints_array = np.concatenate(
                [waypoints.T, target_speeds.reshape(1, -1)]
            )

            send_output(
                "waypoints",
                waypoints_array.tobytes(),
            )
        return DoraStatus.CONTINUE

    def run(self, _ttd=None):
        """Runs the planner.

        Note:
            The planner assumes that the world is up-to-date.

        Returns:
            :py:class:`~pylot.planning.waypoints.Waypoints`: Waypoints of the
            planned trajectory.
        """

        # Remove already past waypoints for gps
        (index, _) = closest_vertex(
            self.gps_waypoints,
            np.array([self.position[:2]]),
        )

        self.gps_waypoints = self.gps_waypoints[
            index : index + NUM_WAYPOINTS_AHEAD
        ]
        obstacle_list = get_obstacle_list(self.obstacles, self.gps_waypoints)

        if len(obstacle_list) == 0:
            # Do not use Hybrid A* if there are no obstacles.
            speeds = np.array([TARGET_SPEED] * len(self.gps_waypoints))
            return self.gps_waypoints, speeds

        # Remove already past waypoints
        (index, _) = closest_vertex(
            self.waypoints,
            np.array([self.position[:2]]),
        )

        self.waypoints = self.waypoints[index : index + NUM_WAYPOINTS_AHEAD]
        obstacle_list = get_obstacle_list(self.obstacles, self.waypoints)

        if len(obstacle_list) == 0:
            # Do not use Hybrid A* if there are no obstacles.
            speeds = np.array([TARGET_SPEED] * len(self.waypoints))
            return self.waypoints, speeds
        # Hybrid a* does not take into account the driveable region.
        # It constructs search space as a top down, minimum bounding
        # rectangle with padding in each dimension.

        initial_conditions = self._compute_initial_conditions(obstacle_list)

        path_x, path_y, _, success = apply_hybrid_astar(
            initial_conditions, self._hyperparameters
        )

        if not success:
            print("could not find waypoints")
            speeds = np.array([0] * len(self.waypoints))
            return self.waypoints, speeds
        else:
            output_wps = np.array([path_x, path_y]).T
            speeds = np.array([TARGET_SPEED] * len(path_x))

            return output_wps, speeds

    def _compute_initial_conditions(self, obstacle_list):
        [x, y, _, _, yaw, _, _] = self.position
        start = np.array(
            [
                x,
                y,
                np.deg2rad(yaw),
            ]
        )
        end_index = min(
            NUM_WAYPOINTS_AHEAD,
            len(self.waypoints) - 1,
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
            [end_x, end_y] = self.waypoints[end_index]
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
            "obs": obstacle_list,
        }
        return initial_conditions
