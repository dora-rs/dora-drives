from dora_utils import (
    LABELS,
    DoraStatus,
    get_extrinsic_matrix,
    get_intrinsic_matrix,
    get_projection_matrix,
    location_to_camera_view,
    local_points_to_camera_view,
    to_world_coordinate,
)

import numpy as np
from typing import Callable
from scipy.spatial.transform import Rotation as R


CAMERA_WIDTH = 800
CAMERA_HEIGHT = 600
DEPTH_IMAGE_WIDTH = 800
DEPTH_IMAGE_HEIGHT = 600
DEPTH_FOV = 90
SENSOR_POSITION = np.array([3, 0, 1, 0, 0, 0])
INTRINSIC_MATRIX = get_intrinsic_matrix(
    DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, DEPTH_FOV
)

INV_INTRINSIC_MATRIX = np.linalg.inv(INTRINSIC_MATRIX)
VELODYNE_MATRIX = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
INV_VELODYNE_MATRIX = np.linalg.inv(VELODYNE_MATRIX)


def get_predictions(obstacles, obstacle_with_locations):
    """Extracts obstacle predictions out of the message.
    This method is useful to build obstacle predictions when
    the operator directly receives detections instead of predictions.
    The method assumes that the obstacles are static.
    """
    predictions = []
    # Transform the obstacle into a prediction.
    for obstacle, location in zip(obstacles, obstacle_with_locations):
        obstacle = np.append(location, obstacle[-2:])
        predictions.append(obstacle)

    return predictions


class Operator:
    """
    Compute the location of obstacles.
    """

    def __init__(self):
        self.point_cloud = []
        self.obstacles = []
        self.position = []

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        if "lidar_pc" == dora_input["id"]:
            point_cloud = np.frombuffer(dora_input["data"], dtype=np.float32)
            point_cloud = point_cloud.reshape((-1, 3))

            point_cloud = np.dot(
                point_cloud,
                np.array([[0, 0, 1], [1, 0, 0], [0, -1, 0]]),
            )
            point_cloud = point_cloud[np.where(point_cloud[:, 2] > 0.1)]
            point_cloud = local_points_to_camera_view(
                point_cloud, INTRINSIC_MATRIX
            )

            if len(point_cloud) != 0:
                self.point_cloud = point_cloud.T
        elif "position" == dora_input["id"]:
            # Add sensor transform
            self.position = np.frombuffer(dora_input["data"], np.float32)
        elif "obstacles_bbox" == dora_input["id"]:
            if len(self.position) == 0 or len(self.point_cloud) == 0:
                return DoraStatus.CONTINUE

            self.obstacles_bbox = np.frombuffer(
                dora_input["data"], dtype="int32"
            ).reshape((-1, 6))

            obstacles_with_location = []
            for obstacle_bb in self.obstacles_bbox:
                [min_x, max_x, min_y, max_y, confidence, label] = obstacle_bb
                z_points = self.point_cloud[
                    np.where(
                        (self.point_cloud[:, 0] > min_x)
                        & (self.point_cloud[:, 0] < max_x)
                        & (self.point_cloud[:, 1] > min_y)
                        & (self.point_cloud[:, 1] < max_y)
                    )
                ]
                if len(z_points) > 0:
                    closest_point = z_points[
                        z_points[:, 2].argsort()[int(len(z_points) / 4)]
                    ]
                    obstacles_with_location.append(closest_point)
            if len(obstacles_with_location) > 0:
                obstacles_with_location = np.array(obstacles_with_location)
                obstacles_with_location = np.dot(
                    INV_INTRINSIC_MATRIX, obstacles_with_location.T
                ).T

                obstacles_with_location = np.dot(
                    obstacles_with_location, INV_VELODYNE_MATRIX
                )
                [x, y, z, rx, ry, rz, rw] = self.position
                rot = R.from_quat([rx, ry, rz, rw])
                obstacles_with_location = rot.apply(obstacles_with_location) + [
                    x,
                    y,
                    z,
                ]
                predictions = get_predictions(
                    self.obstacles_bbox, obstacles_with_location
                )
                predictions_bytes = np.array(
                    predictions, dtype="float32"
                ).tobytes()

                send_output(
                    "obstacles", predictions_bytes, dora_input["metadata"]
                )
            else:
                send_output(
                    "obstacles", np.array([]).tobytes(), dora_input["metadata"]
                )
        return DoraStatus.CONTINUE
