import logging

import numpy as np
import numpy.matlib

from dora_utils import (
    closest_vertex,
    get_extrinsic_matrix,
    get_intrinsic_matrix,
    get_projection_matrix,
    to_world_coordinate,
)

logger = logging.Logger("Obstacle Location")
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
    frame = depth_frame.astype(np.float32)
    frame = np.reshape(
        frame, (DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_CHANNEL)
    )
    # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).
    frame = np.dot(frame[:, :, :3], [65536.0, 256.0, 1.0])
    frame /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)

    # 2d pixel coordinates
    u_coord = np.matlib.repmat(
        np.r_[0:DEPTH_IMAGE_WIDTH:1], DEPTH_IMAGE_HEIGHT, 1
    ).reshape(PIXEL_LENGTH)
    v_coord = np.matlib.repmat(
        np.c_[0:DEPTH_IMAGE_HEIGHT:1], 1, DEPTH_IMAGE_WIDTH
    ).reshape(PIXEL_LENGTH)
    normalized_depth = np.reshape(frame, PIXEL_LENGTH)

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
        obstacle_bytes = np.array([location]).tobytes()
        obstacle_bytes += obstacle[-2:].tobytes()

    predictions_bytes = b"\n".join(predictions)
    return predictions_bytes


def get_obstacle_locations(
    obstacles,
    depth_frame,
    ego_position: np.array,
    depth_frame_position: np.array,
):

    depth_frame_matrix = np.dot(
        get_projection_matrix(ego_position),
        get_projection_matrix(depth_frame_position),
    )

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

        locations = point_cloud[sample_points]

        # Choose the closest from the locations of the sampled points.

        min_location = closest_vertex(locations, [ego_position[:3]])

        obstacle_with_locations.append(locations[min_location])

    return obstacle_with_locations


def dora_run(inputs):
    keys = inputs.keys()

    if (
        "depth_frame" not in keys
        or "obstacles_without_location" not in keys
        or "position" not in keys
    ):
        return {}

    obstacles = np.frombuffer(
        inputs["obstacles_without_location"], dtype="float32"
    )
    obstacles = obstacles.reshape((-1, 12))

    buffer_depth_frame = inputs["depth_frame"]
    depth_frame = np.frombuffer(
        buffer_depth_frame[: 800 * 600 * 4], dtype="uint8"
    )
    depth_frame_position = np.frombuffer(
        buffer_depth_frame[800 * 600 * 4 :], dtype="float32"
    )

    position = np.frombuffer(inputs["position"])

    obstacles_with_location = get_obstacle_locations(
        obstacles, depth_frame, position, depth_frame_position
    )

    predictions_bytes = get_predictions(obstacles, obstacles_with_location)

    return {"obstacles": predictions_bytes}
