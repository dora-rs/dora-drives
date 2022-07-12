import logging
import math

import numpy as np
import pylot.utils
from pylot.perception.tracking.obstacle_trajectory import ObstacleTrajectory
from pylot.prediction.obstacle_prediction import ObstaclePrediction

from dora_watermark import dump, load

logger = logging.Logger("Obstacle Location")


def get_predictions(obstacles, position_matrix):
    """Extracts obstacle predictions out of the message.
    This method is useful to build obstacle predictions when
    the operator directly receives detections instead of predictions.
    The method assumes that the obstacles are static.
    """
    predictions = []
    # Transform the obstacle into a prediction.
    for obstacle in obstacles:
        obstacle_trajectory = ObstacleTrajectory(obstacle, [])
        inverse_matrix = np.linalg.inv(position_matrix)
        prediction = ObstaclePrediction(
            obstacle_trajectory,
            obstacle.transform,
            1.0,
            [np.dot(inverse_matrix, obstacle.transform.matrix)],
        )
        predictions.append(prediction)

    return predictions


def get_obstacle_locations(
    obstacles,
    depth_frame,
    ego_transform,
):

    depth_frame.camera_setup.set_transform(
        ego_transform * depth_frame.camera_setup.transform
    )

    for obstacle in obstacles:
        center_point = obstacle.bounding_box_2D.get_center_point()
        # Sample several points around the center of the bounding box
        # in case the bounding box is not well centered on the obstacle.
        # In such situations the center point might be in between legs,
        # and thus we might overestimate the distance.
        sample_points = []
        for delta_x in range(-30, 30, 5):
            for delta_y in range(-30, 30, 5):
                sample_point = center_point + pylot.utils.Vector2D(
                    delta_x, delta_y
                )
                if obstacle.bounding_box_2D.is_within(sample_point):
                    sample_points.append(sample_point)
        locations = depth_frame.get_pixel_locations(sample_points)
        # Choose the closest from the locations of the sampled points.
        min_distance = np.infty
        closest_location = None
        for location in locations:

            dist = location.distance(ego_transform.location)
            if dist < min_distance:
                min_distance = dist
                closest_location = location
        obstacle.transform = pylot.utils.Transform(
            closest_location, pylot.utils.Rotation()
        )
    return obstacles


def create_matrix(position: np.array):
    """Creates a transformation matrix to convert points in the 3D world
    coordinate space with respect to the object.
    Use the transform_points function to transpose a given set of points
    with respect to the object.
    Args:
        location (:py:class:`.Location`): The location of the object
            represented by the transform.
        rotation (:py:class:`.Rotation`): The rotation of the object
            represented by the transform.
    Returns:
        A 4x4 numpy matrix which represents the transformation matrix.
    """
    matrix = np.identity(4)
    cy = np.cos(np.radians(position[4]))
    sy = np.sin(np.radians(position[4]))
    cr = np.cos(np.radians(position[5]))
    sr = np.sin(np.radians(position[5]))
    cp = np.cos(np.radians(position[3]))
    sp = np.sin(np.radians(position[3]))
    matrix[:3, 3] = position[:3]
    matrix[0, 0] = cp * cy
    matrix[0, 1] = cy * sp * sr - sy * cr
    matrix[0, 2] = -1 * (cy * sp * cr + sy * sr)
    matrix[1, 0] = sy * cp
    matrix[1, 1] = sy * sp * sr + cy * cr
    matrix[1, 2] = cy * sr - sy * sp * cr
    matrix[2, 0] = sp
    matrix[2, 1] = -1 * (cp * sr)
    matrix[2, 2] = cp * cr
    return matrix


def dora_run(inputs):
    keys = inputs.keys()

    if (
        "depth_frame" not in keys
        or "obstacles_without_location" not in keys
        or "position" not in keys
    ):
        return {}

    obstacles = load(inputs, "obstacles_without_location")
    depth_frame = load(inputs, "depth_frame")
    position = np.frombuffer(inputs["position"])

    [x, y, z, pitch, yaw, roll, current_speed] = position
    pose = pylot.utils.Pose(
        pylot.utils.Transform(
            pylot.utils.Location(x, y, z),
            pylot.utils.Rotation(pitch, yaw, roll),
        ),
        current_speed,
    )

    position_matrix = create_matrix(position)

    obstacles_with_location = get_obstacle_locations(
        obstacles,
        depth_frame,
        pose.transform,
    )

    obstacles_with_prediction = get_predictions(
        obstacles_with_location, position_matrix
    )

    obstacles = dump(obstacles_with_prediction)

    return {"obstacles": obstacles}
