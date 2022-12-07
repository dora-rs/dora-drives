from typing import Callable
import time
import numpy as np
from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory import (
    fot_wrapper,
)
from dora_utils import DoraStatus, closest_vertex, pairwise_distances
from scipy.spatial.transform import Rotation as R
import os


# Planning general
TARGET_SPEED = 5
NUM_WAYPOINTS_AHEAD = 10
GOAL_WAYPOINTS = os.environ.get("GOAL_WAYPOINTS")
if GOAL_WAYPOINTS:
    GOAL_LOCATION = np.fromstring(GOAL_WAYPOINTS, dtype=float, sep=",")
else:
    GOAL_LOCATION = np.array([[0, 0], [0, 1.5], [3, 2.5]])
OBSTACLE_CLEARANCE = 10
OBSTACLE_RADIUS = 0.5


def get_obstacle_list(obstacle_predictions, waypoints):
    if len(obstacle_predictions) == 0 or len(waypoints) == 0:
        return np.empty((0, 4))
    obstacle_list = []

    distances = pairwise_distances(waypoints, obstacle_predictions[:, :2]).min(
        0
    )
    for distance, prediction in zip(distances, obstacle_predictions):
        # Use all prediction times as potential obstacles.
        if distance < OBSTACLE_CLEARANCE:
            [x, y, _, _confidence, _label] = prediction
            obstacle_size = np.array(
                [
                    x,
                    y,
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
        self.obstacles = np.array([])
        self.position = []
        self.waypoints = []
        self.gps_waypoints = []
        self.obstacle_metadata = {}
        self.gps_metadata = {}
        self.metadata = {}
        self.orientation = None
        self.outputs = []
        self.hyperparameters = {
            "max_speed": 25.0,
            "max_accel": 15.0,
            "max_curvature": 15.0,
            "max_road_width_l": 5.0,
            "max_road_width_r": 5.0,
            "d_road_w": 0.5,
            "dt": 0.2,
            "maxt": 5.0,
            "mint": 2.0,
            "d_t_s": 0.5,
            "n_s_sample": 2.0,
            "obstacle_clearance": 0.1,
            "kd": 1.0,
            "kv": 0.1,
            "ka": 0.1,
            "kj": 0.1,
            "kt": 0.1,
            "ko": 0.1,
            "klat": 1.0,
            "klon": 1.0,
            "num_threads": 0,  # set 0 to avoid using threaded algorithm
        }
        self.conds = {
            "s0": 0,
            "target_speed": TARGET_SPEED,
        }  # paste output from debug log

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):

        if dora_input["id"] == "position":
            self.position = np.frombuffer(dora_input["data"], np.float32)

        elif dora_input["id"] == "obstacles":
            obstacles = np.frombuffer(
                dora_input["data"], dtype="float32"
            ).reshape((-1, 5))
            self.obstacles = obstacles
            return DoraStatus.CONTINUE

        if "gps_waypoints" == dora_input["id"]:
            waypoints = np.frombuffer(dora_input["data"], np.float32)
            waypoints = waypoints.reshape((-1, 3))[:, :2]
            self.gps_waypoints = waypoints
            return DoraStatus.CONTINUE

        if len(self.gps_waypoints) == 0:
            print("No waypoints")
            return DoraStatus.CONTINUE

        elif len(self.position) == 0:
            print("No position")
            return DoraStatus.CONTINUE

        [x, y, z, rx, ry, rz, rw] = self.position
        [_, _, yaw] = R.from_quat([rx, ry, rz, rw]).as_euler(
            "xyz", degrees=False
        )

        gps_obstacles = get_obstacle_list(self.obstacles, self.gps_waypoints)

        initial_conditions = {
            "ps": 0,
            "target_speed": self.conds["target_speed"],
            "pos": self.position[:2],
            "vel": 0.5 * np.array([np.cos(yaw), np.sin(yaw)]),
            "wp": self.gps_waypoints,
            "obs": gps_obstacles,
        }

        (
            result_x,
            result_y,
            speeds,
            ix,
            iy,
            iyaw,
            d,
            s,
            speeds_x,
            speeds_y,
            misc,
            costs,
            success,
        ) = fot_wrapper.run_fot(initial_conditions, self.hyperparameters)
        if not success:
            print(f"fot failed. stopping with {initial_conditions}.")
            send_output("waypoints", np.array([]).tobytes())
            return DoraStatus.STOP

        self.waypoints = np.concatenate([result_x, result_y]).reshape((2, -1)).T

        self.outputs = (
            np.concatenate([result_x, result_y, speeds])
            .reshape((3, -1))
            .T.astype(np.float32)
        )
        send_output("waypoints", self.outputs.tobytes(), dora_input["metadata"])
        return DoraStatus.CONTINUE
