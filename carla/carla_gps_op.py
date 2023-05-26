from typing import Callable

import numpy as np
import pyarrow as pa
from _dora_utils import closest_vertex
from _hd_map import HDMap
from dora import DoraStatus
from scipy.spatial.transform import Rotation as R

pa.array([])
from carla import Map

# Planning general
NUM_WAYPOINTS_AHEAD = 120
GOAL_LOCATION = [234, 59, 39]


def filter_consecutive_duplicate(x):
    return np.array(
        [elem for i, elem in enumerate(x) if (elem - x[i - 1]).any()]
    )


class Operator:
    """
    Compute GPS waypoints, given a `position`, an `objective waypoints` and an `opendrive` map.
    """

    def __init__(self):
        self._goal_location = GOAL_LOCATION
        self.hd_map = None
        self.position = []
        self.waypoints = np.array([])
        self.target_speeds = np.array([])
        self.objective_waypoints = []
        self.waypoints_array = np.array([])

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

        if "position" == dora_input["id"]:
            self.position = np.array(dora_input["value"]).view(np.float32)
            return DoraStatus.CONTINUE

        elif "opendrive" == dora_input["id"]:
            # Registering the map
            opendrive = dora_input["data"].decode()
            self.hd_map = HDMap(Map("map", opendrive))
            return DoraStatus.CONTINUE

        elif "objective_waypoints" == dora_input["id"]:
            self.objective_waypoints = (
                np.array(dora_input["value"]).view(np.float32).reshape((-1, 3))
            )

            if self.hd_map is None or len(self.position) == 0:
                print("No map within the gps or position")
                return DoraStatus.CONTINUE

            (index, _closest_objective) = closest_vertex(
                self.objective_waypoints[:, :2],
                np.array([self.position[:2]]),
            )

            self.objective_waypoints = self.objective_waypoints[
                index : index + NUM_WAYPOINTS_AHEAD
            ]
            self._goal_location = self.objective_waypoints[0]

            # Used cached waypoints but start at closest point.
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

            elif len(self.waypoints) == 0:

                # Deconstructing the position
                [x, y, z, rx, ry, rz, rw] = self.position
                [pitch, roll, yaw] = R.from_quat([rx, ry, rz, rw]).as_euler(
                    "xyz", degrees=False
                )

                # Compute the waypoints
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
                # if np.abs(diff_angle) > np.pi / 2:
                # print("Error in computation of waypoints.")
                # print(
                # "The next target waypoint requires to make a 180 degrees turn."
                # )
                # print(f"target waypoint: {waypoints[0]}")
                # print(f"position: {[x, y, z]}")
                # print(f"goal location: {self._goal_location}")
                # else:
                self.waypoints = waypoints
                self.target_speeds = np.array([5.0] * len(waypoints))

            if len(self.waypoints) == 0:
                send_output(
                    "gps_waypoints",
                    pa.array(self.waypoints.view(np.uint8).ravel()),
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
                pa.array(
                    filter_consecutive_duplicate(self.waypoints_array)
                    .view(np.uint8)
                    .ravel()
                ),
                dora_input["metadata"],
            )  # World coordinate

            return DoraStatus.CONTINUE
