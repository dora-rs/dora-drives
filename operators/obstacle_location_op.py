import zlib
from typing import Callable

import numpy as np
import numpy.matlib

from dora_utils import (
    DoraStatus,
    closest_vertex,
    get_extrinsic_matrix,
    get_intrinsic_matrix,
    get_projection_matrix,
    to_world_coordinate,
)

DEPTH_CAMERA_MAX_DEPTH = 1000
DEPTH_IMAGE_WIDTH = 800
DEPTH_IMAGE_HEIGHT = 600
DEPTH_IMAGE_CHANNEL = 4
PIXEL_LENGTH = DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT
DEPTH_FOV = 90
INTRINSIC_MATRIX = get_intrinsic_matrix(
    DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, DEPTH_FOV
)
INV_INTRINSIC_MATRIX = np.linalg.inv(INTRINSIC_MATRIX)


def get_closest_point_in_point_cloud(
    point_cloud: np.array, point: np.array, normalized: bool = False
):
    """Finds the closest point in the point cloud to the given point."""
    # Select x and y.
    pc_xy = point_cloud[:, 0:2]

    # Compute distance
    if normalized:
        # Select z
        pc_z = point_cloud[:, 2]
        # Divize x, y by z
        normalized_pc = pc_xy / pc_z[:, None]
    else:
        normalized_pc = pc_xy

    # Select index of the closest point.
    (closest_point, _) = closest_vertex(normalized_pc, point)

    # Return the closest point.
    return point_cloud[closest_point]


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


def get_obstacle_locations(
    obstacles,
    point_cloud: np.array,
):
    point_cloud = point_cloud[np.where(point_cloud[:, 2] > 0.1)]
    obstacle_with_locations = []
    for obstacle in obstacles:
        # Sample several points around the center of the bounding box
        # in case the bounding box is not well centered on the obstacle.
        # In such situations the center point might be in between legs,
        # and thus we might overestimate the distance.

        obstacle_center = np.array(
            [
                [
                    (obstacle[1] + obstacle[0]) / 2,
                    (obstacle[3] + obstacle[2]) / 2,
                    1.0,
                ]
            ]
        )

        # Project our 2D pixel location into 3D space, onto the z=1 plane.
        p3d = np.dot(
            INV_INTRINSIC_MATRIX, obstacle_center.transpose()
        ).transpose()

        # Choose the closest from the locations of the sampled points.
        min_location = get_closest_point_in_point_cloud(
            point_cloud, p3d[:, :2], True
        )
        p3d *= min_location[2]

        obstacle_with_locations.append(min_location)

    return obstacle_with_locations


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.point_cloud = []
        self.obstacles = []
        self.position = []

    def on_input(
        self,
        input: dict,
        send_output: Callable[[str, bytes], None],
    ):

        if input["id"] == "position":
            # Add sensor transform
            self.position = np.frombuffer(input["data"])[:6] + np.array(
                [3.0, 0, 1, 0, 0, 0]
            )

        elif input["id"] == "lidar_pc":
            point_cloud = np.frombuffer(
                zlib.decompress(input["data"]), dtype=np.dtype("f4")
            )
            point_cloud = np.reshape(
                point_cloud, (int(point_cloud.shape[0] / 4), 4)
            )

            # To camera coordinate
            # The latest coordinate space is the unreal space.
            point_cloud = np.dot(
                point_cloud,
                np.array(
                    [[0, 1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]]
                ),
            )
            self.point_cloud = point_cloud[:, :3]

        elif (
            input["id"] == "obstacles_bbox"
            and len(self.point_cloud) != 0
            and len(self.position) != 0
        ):
            if len(input["data"]) == 0:
                send_output("obstacles", np.array([]).tobytes())
                return DoraStatus.CONTINUE
            obstacles = (
                np.frombuffer(input["data"], dtype="int32")
                .reshape((-1, 6))
                .astype(np.float32)
            )
            obstacles_with_location = get_obstacle_locations(
                obstacles, self.point_cloud
            )

            projection_matrix = get_projection_matrix(self.position)
            extrinsic_matrix = get_extrinsic_matrix(projection_matrix)
            obstacles_with_location = to_world_coordinate(
                np.array(obstacles_with_location), extrinsic_matrix
            )

            predictions = get_predictions(obstacles, obstacles_with_location)
            predictions_bytes = np.array(predictions, dtype="float32").tobytes()

            send_output("obstacles", predictions_bytes, input["metadata"])
        return DoraStatus.CONTINUE
