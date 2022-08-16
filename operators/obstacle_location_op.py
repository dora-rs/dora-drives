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


def get_point_cloud(depth_frame, depth_frame_matrix):
    """Converts the depth frame to a 1D array containing the 3D
    position of each pixel in world coordinates.
    """

    # 2d pixel coordinates
    u_coord = np.matlib.repmat(
        np.r_[0:DEPTH_IMAGE_WIDTH:1], DEPTH_IMAGE_HEIGHT, 1
    ).reshape(PIXEL_LENGTH)
    v_coord = np.matlib.repmat(
        np.c_[0:DEPTH_IMAGE_HEIGHT:1], 1, DEPTH_IMAGE_WIDTH
    ).reshape(PIXEL_LENGTH)
    normalized_depth = np.reshape(depth_frame, PIXEL_LENGTH)

    # p2d = [u,v,1]
    p2d = np.array([u_coord, v_coord, np.ones_like(u_coord)])

    # P = [X,Y,Z]
    p3d = np.dot(np.linalg.inv(INTRINSIC_MATRIX), p2d)
    p3d *= normalized_depth * DEPTH_CAMERA_MAX_DEPTH

    # [[X1,Y1,Z1],[X2,Y2,Z2], ... [Xn,Yn,Zn]]
    locations = np.asarray(np.transpose(p3d))
    # Transform the points in 3D world coordinates.
    extrinsic_matrix = get_extrinsic_matrix(depth_frame_matrix)
    point_cloud = to_world_coordinate(locations, extrinsic_matrix)
    return point_cloud


def get_predictions(obstacles, obstacle_with_locations):
    """Extracts obstacle predictions out of the message.
    This method is useful to build obstacle predictions when
    the operator directly receives detections instead of predictions.
    The method assumes that the obstacles are static.
    """
    predictions = []
    # Transform the obstacle into a prediction.
    for obstacle, location in zip(obstacles, obstacle_with_locations):
        obstacle_bytes = np.array(location, dtype="float32").tobytes()
        obstacle_bytes += obstacle[-2:].tobytes()
        predictions.append(obstacle_bytes)

    return predictions


def get_obstacle_locations(
    obstacles,
    depth_frame,
    depth_frame_position: np.array,
):

    depth_frame_matrix = get_projection_matrix(depth_frame_position)

    point_cloud = get_point_cloud(depth_frame, depth_frame_matrix)

    obstacle_with_locations = []
    for obstacle in obstacles:
        # Sample several points around the center of the bounding box
        # in case the bounding box is not well centered on the obstacle.
        # In such situations the center point might be in between legs,
        # and thus we might overestimate the distance.
        sample_points = []
        coefficient = (
            (obstacle[1] - obstacle[0]) / 2,
            (obstacle[3] - obstacle[2]) / 2,
        )
        left_corner = (
            (obstacle[1] + obstacle[0]) / 2,
            (obstacle[3] + obstacle[2]) / 2,
        )
        sample_points = (
            np.random.uniform(size=(32, 2)) * coefficient + left_corner
        )
        sample_points = sample_points.astype(int)

        locations = np.array(
            [
                point_cloud[point[1] * DEPTH_IMAGE_WIDTH + point[0]]
                for point in sample_points
                if point[1] > 0
                and point[0] > 0
                and (point[1] * DEPTH_IMAGE_WIDTH + point[0])
                < (DEPTH_IMAGE_HEIGHT * DEPTH_IMAGE_WIDTH)
            ]
        )

        # Choose the closest from the locations of the sampled points.
        if len(locations) > 0:
            _, min_location = closest_vertex(
                locations, depth_frame_position[:3].reshape((1, -1))
            )

            obstacle_with_locations.append(min_location)

    return obstacle_with_locations


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.depth_frame = []
        self.depth_frame_position = []
        self.obstacles = []

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ):

        if input_id == "depth_frame":
            depth_frame = np.frombuffer(
                zlib.decompress(value[24:]),
                dtype="float32",
            )
            depth_frame = np.reshape(
                depth_frame, (DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH)
            )

            self.depth_frame = depth_frame
            self.depth_frame_position = np.frombuffer(
                value[:24],
                dtype="float32",
            )
            return DoraStatus.CONTINUE

        if (
            input_id == "obstacles_without_location"
            and len(self.depth_frame) != 0
        ):
            if len(value) == 0:
                return DoraStatus.CONTINUE
            obstacles = np.frombuffer(value, dtype="int32").reshape((-1, 6))
            self.obstacles = np.array(obstacles, dtype=np.float32)
            obstacles_with_location = get_obstacle_locations(
                obstacles, self.depth_frame, self.depth_frame_position
            )

            predictions = get_predictions(obstacles, obstacles_with_location)
            predictions_bytes = b"\n".join(predictions)
            send_output("obstacles", predictions_bytes)
        return DoraStatus.CONTINUE
