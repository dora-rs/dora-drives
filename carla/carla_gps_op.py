import threading
from typing import Callable

import numpy as np

from _dora_utils import DoraStatus, closest_vertex
from _hd_map import HDMap
from carla import Client, Map
from numpy import linalg as LA
from scipy.spatial.transform import Rotation as R

mutex = threading.Lock()


# Planning general
TARGET_SPEED = 7.0
NUM_WAYPOINTS_AHEAD = 120
GOAL_LOCATION = [234, 59, 39]
CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"
OBJECTIVE_MIN_DISTANCE = 20


def filter_consecutive_duplicate(x):
    return np.array(
        [elem for i, elem in enumerate(x) if (elem - x[i - 1]).any()]
    )


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self._goal_location = GOAL_LOCATION
        client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
        client.set_timeout(50.0)  # seconds
        self.client = client
        carla_world = client.get_world()
        self.carla_world_id = carla_world.id
        hd_map = HDMap(carla_world.get_map())
        self.hd_map = hd_map
        self.position = []
        self.waypoints = np.array([])
        self.target_speeds = np.array([])
        self.objective_waypoints = []
        self.completed_waypoints = 0
        self.waypoints_array = np.array([])

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):

        if "position" == dora_input["id"]:
            self.position = np.frombuffer(dora_input["data"], np.float32)

            return DoraStatus.CONTINUE

        # if "opendrive" == dora_input["id"]:
        # opendrive = dora_input["data"].decode()
        # self.hd_map = HDMap(Map("map", opendrive))

        if "objective_waypoints" == dora_input["id"]:
            self.objective_waypoints = np.frombuffer(
                dora_input["data"], np.float32
            ).reshape((-1, 3))[self.completed_waypoints :]
            # carla_world = self.client.get_world()
            # if self.carla_world_id != carla_world.id:
            # hd_map = HDMap(carla_world.get_map())
            # self.hd_map = hd_map
            # self.waypoints = []
            # self.target_speeds = []
            (index, closest_objective) = closest_vertex(
                self.objective_waypoints[:, :2],
                np.array([self.position[:2]]),
            )

            if (
                LA.norm(closest_objective - self.position[:2])
                < OBJECTIVE_MIN_DISTANCE
            ):
                self.completed_waypoints += 1

            self.objective_waypoints = self.objective_waypoints[
                index : index + NUM_WAYPOINTS_AHEAD
            ]
            self._goal_location = self.objective_waypoints[0]

            if len(self.waypoints) != 0:
                (index, _) = closest_vertex(
                    self.waypoints,
                    np.array([self.position[:2]]),
                )

                self.waypoints = self.waypoints[
                    index : index + NUM_WAYPOINTS_AHEAD
                ]
                self.target_speeds = self.target_speeds[
                    index : index + NUM_WAYPOINTS_AHEAD
                ]

            if len(self.waypoints) < NUM_WAYPOINTS_AHEAD / 2:

                carla_world = self.client.get_world()
                if self.carla_world_id != carla_world.id:
                    hd_map = HDMap(carla_world.get_map())
                    self.hd_map = hd_map
                    self.carla_world_id = carla_world.id

                [x, y, z, rx, ry, rz, rw] = self.position
                [pitch, roll, yaw] = R.from_quat([rx, ry, rz, rw]).as_euler(
                    "xyz", degrees=False
                )

                waypoints = self.hd_map.compute_waypoints(
                    [
                        x,
                        y,
                        self._goal_location[2],
                    ],
                    self._goal_location,
                )[:NUM_WAYPOINTS_AHEAD]

                ## Verify that computed waypoints are not inverted
                target_vector = waypoints[0] - self.position[:2]
                angle = np.arctan2(target_vector[1], target_vector[0])
                diff_angle = np.arctan2(
                    np.sin(angle - yaw), np.cos(angle - yaw)
                )
                if np.abs(diff_angle) > np.pi * 2 / 3:
                    print("Error in computation of waypoints")
                    print(f"target waypoint: {waypoints[0]}")
                    print(f"position: {[x, y, z]}")
                    print(f"goal location: {self._goal_location}")
                else:
                    self.waypoints = waypoints
                    self.target_speeds = np.array([5.0] * len(waypoints))

            if len(self.waypoints) == 0:
                send_output(
                    "gps_waypoints",
                    self.waypoints.tobytes(),
                    dora_input["metadata"],
                )  # World coordinate
                return DoraStatus.CONTINUE

            self.waypoints_array = np.concatenate(
                [
                    self.waypoints.T,
                    self.target_speeds.reshape(1, -1),
                ]
            ).T.astype(np.float32)

            send_output(
                "gps_waypoints",
                filter_consecutive_duplicate(self.waypoints_array).tobytes(),
                dora_input["metadata"],
            )  # World coordinate

        return DoraStatus.CONTINUE
