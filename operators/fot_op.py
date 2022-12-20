from typing import Callable
import time
import numpy as np
from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory import (
    fot_wrapper,
)
from dora_utils import DoraStatus, closest_vertex, pairwise_distances, LABELS
from scipy.spatial.transform import Rotation as R
import os


# Planning general
TARGET_SPEED = 10
NUM_WAYPOINTS_AHEAD = 10

OBSTACLE_CLEARANCE = 10
OBSTACLE_RADIUS = 3
OBSTACLE_RADIUS_TANGENT = 3
MAX_CURBATURE = np.pi / 4


def get_obstacle_list(position, obstacle_predictions, waypoints):

    [x_ego, y_ego, z, rx, ry, rz, rw] = position
    [pitch, roll, yaw] = R.from_quat([rx, ry, rz, rw]).as_euler(
        "xyz", degrees=False
    )

    if len(obstacle_predictions) == 0 or len(waypoints) == 0:
        return np.empty((0, 4))
    obstacle_list = []

    distances = pairwise_distances(waypoints, obstacle_predictions[:, :2]).min(
        0
    )
    for distance, prediction in zip(distances, obstacle_predictions):
        # Use all prediction times as potential obstacles.
        [x, y, _, _confidence, _label] = prediction
        angle = np.arctan2(y - y_ego, x - x_ego)
        diff_angle = (yaw - angle) % 2 * np.pi

        if distance < OBSTACLE_CLEARANCE and diff_angle < MAX_CURBATURE:
            obstacle_size = np.array(
                [
                    x - OBSTACLE_RADIUS_TANGENT * np.cos(angle) / 2,
                    y - OBSTACLE_RADIUS_TANGENT * np.sin(angle) / 2,
                    x
                    + OBSTACLE_RADIUS * np.cos(angle)
                    + OBSTACLE_RADIUS_TANGENT * np.cos(angle) / 2,
                    y
                    + OBSTACLE_RADIUS * np.sin(angle)
                    + OBSTACLE_RADIUS_TANGENT * np.sin(angle) / 2,
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
            "max_road_width_l": 0.2,
            "max_road_width_r": 0.2,
            "d_road_w": 0.2,
            "dt": 0.1,
            "maxt": 5.0,
            "mint": 2.0,
            "d_t_s": 5,
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
            return DoraStatus.CONTINUE

        elif dora_input["id"] == "obstacles":
            obstacles = np.frombuffer(
                dora_input["data"], dtype="float32"
            ).reshape((-1, 5))
            self.obstacles = obstacles
            return DoraStatus.CONTINUE

        elif "gps_waypoints" == dora_input["id"]:
            waypoints = np.frombuffer(dora_input["data"], np.float32)
            waypoints = waypoints.reshape((-1, 3))[:, :2]
            self.gps_waypoints = waypoints

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

        gps_obstacles = get_obstacle_list(
            self.position, self.obstacles, self.gps_waypoints
        )

        initial_conditions = {
            "ps": 0,
            "target_speed": self.conds["target_speed"],
            "pos": self.position[:2],
            "vel": 10 * np.array([np.cos(yaw), np.sin(yaw)]),
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
            for obstacle in self.obstacles:
                print(
                    f"obstacles:{obstacle}, label: {LABELS[int(obstacle[-1])]}"
                )
            send_output(
                "waypoints", np.array([x, y, 0.0], np.float32).tobytes()
            )
            return DoraStatus.CONTINUE

        self.waypoints = np.concatenate([result_x, result_y]).reshape((2, -1)).T

        self.outputs = (
            np.concatenate([result_x, result_y, speeds])
            .reshape((3, -1))
            .T.astype(np.float32)
        )
        send_output("waypoints", self.outputs.tobytes(), dora_input["metadata"])
        return DoraStatus.CONTINUE
