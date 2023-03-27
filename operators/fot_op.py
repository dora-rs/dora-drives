from typing import Callable

import time
import numpy as np
from dora import DoraStatus
from dora_utils import LABELS, pairwise_distances
from frenet_optimal_trajectory_planner.FrenetOptimalTrajectory import (
    fot_wrapper,
)
from numpy import linalg as LA
from scipy.spatial.transform import Rotation as R

# Planning general
TARGET_SPEED = 7

OBSTACLE_CLEARANCE = 3
OBSTACLE_RADIUS = 1
OBSTACLE_RADIUS_TANGENT = 1.5
MAX_CURBATURE = np.pi / 6

def get_lane_list(position, lanes, waypoints):

    lane_list = []

    lanes_xy = lanes[:, :, :2]

    for lane in lanes_xy:
        lane_list.append(np.concatenate([lane - 0.1, lane + 0.1], axis=1))

    if len(lane_list) > 0:
        return np.concatenate(lane_list)
    else:
        return np.array([])


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
        diff_angle = np.arctan2(np.sin(angle - yaw), np.cos(angle - yaw))

        if distance < OBSTACLE_CLEARANCE and np.abs(diff_angle) < MAX_CURBATURE:
            obstacle_size = np.array(
                [
                    x
                    - OBSTACLE_RADIUS * np.cos(angle) / 2
                    - OBSTACLE_RADIUS_TANGENT * np.sin(angle) / 2,
                    y
                    - OBSTACLE_RADIUS * np.sin(angle) / 2
                    - OBSTACLE_RADIUS_TANGENT * np.cos(angle) / 2,
                    x
                    + OBSTACLE_RADIUS * np.cos(angle) / 2
                    + OBSTACLE_RADIUS_TANGENT * np.sin(angle) / 2,
                    y
                    + OBSTACLE_RADIUS * np.sin(angle) / 2
                    + OBSTACLE_RADIUS_TANGENT * np.cos(angle) / 2,
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
        self.lanes = np.array([])
        self.position = []
        self.speed = []
        self.last_position = []
        self.waypoints = []
        self.gps_waypoints = []
        self.last_obstacles = np.array([])
        self.obstacle_metadata = {}
        self.gps_metadata = {}
        self.metadata = {}
        self.orientation = None
        self.outputs = []
        self.hyperparameters = {
            "max_speed": 25.0,
            "max_accel": 45.0,
            "max_curvature": 55.0,
            "max_road_width_l": 0.1,
            "max_road_width_r": 0.1,
            "d_road_w": 0.5,
            "dt": 0.5,
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

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):

        if dora_input["id"] == "position":
            self.last_position = self.position
            self.position = np.frombuffer(dora_input["data"], np.float32)
            if len(self.last_position) == 0:
                self.last_position = self.position
            return DoraStatus.CONTINUE

        elif dora_input["id"] == "speed":
            self.speed = np.frombuffer(dora_input["data"], np.float32)
            return DoraStatus.CONTINUE

        elif dora_input["id"] == "check":
            send_output("ready", b"")
            return DoraStatus.CONTINUE

        elif dora_input["id"] == "obstacles":
            obstacles = np.frombuffer(
                dora_input["data"], dtype=np.float32
            ).reshape((-1, 5))
            if len(self.last_obstacles) > 0:
                self.obstacles = np.concatenate(
                    [self.last_obstacles, obstacles]
                )
            else:
                self.obstacles = obstacles
            return DoraStatus.CONTINUE

        elif dora_input["id"] == "global_lanes":
            lanes = np.frombuffer(dora_input["data"], dtype=np.float32).reshape(
                (-1, 60, 3)
            )
            self.lanes = lanes
            return DoraStatus.CONTINUE

        elif "gps_waypoints" == dora_input["id"]:
            waypoints = np.frombuffer(dora_input["data"], np.float32)
            waypoints = waypoints.reshape((-1, 3))[:, :2]
            self.gps_waypoints = waypoints

        if len(self.gps_waypoints) == 0:
            print("No waypoints")
            send_output(
                "waypoints",
                self.gps_waypoints.tobytes(),
                dora_input["metadata"],
            )
            return DoraStatus.CONTINUE

        elif len(self.position) == 0:
            print("No position")
            return DoraStatus.CONTINUE

        elif len(self.speed) == 0:
            print("No speed")
            return DoraStatus.CONTINUE
        [x, y, z, rx, ry, rz, rw] = self.position
        [_, _, yaw] = R.from_quat([rx, ry, rz, rw]).as_euler(
            "xyz", degrees=False
        )

        gps_obstacles = get_obstacle_list(
            self.position, self.obstacles, self.gps_waypoints
        )

        if len(self.lanes) > 0:
            lanes = get_lane_list(self.position, self.lanes, self.gps_waypoints)
            obstacles = np.concatenate([gps_obstacles, lanes])
        else:
            obstacles = gps_obstacles
        initial_conditions = {
            "ps": 0,
            "target_speed": self.conds["target_speed"],
            "pos": self.position[:2],
            "vel": (np.clip(LA.norm(self.speed), 0.5, 40))
            * np.array([np.cos(yaw), np.sin(yaw)]),
            "wp": self.gps_waypoints,
            "obs": obstacles,
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
            initial_conditions["wp"] = initial_conditions["wp"][:5]
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
