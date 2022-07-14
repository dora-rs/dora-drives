import logging

import numpy as np
import pylot.utils
from pylot.perception.tracking.obstacle_trajectory import ObstacleTrajectory
from pylot.prediction.obstacle_prediction import ObstaclePrediction

from dora_utils import get_projection_matrix
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
            [
                pylot.utils.Transform(
                    matrix=np.dot(inverse_matrix, obstacle.transform.matrix)
                )
            ],
        )
        predictions.append(prediction)

    return predictions


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

            dist = np.linalg.norm(location - ego_position[:3])
            if dist < min_distance:
                min_distance = dist
                closest_location = location
        obstacle.transform = pylot.utils.Transform(
            closest_location, pylot.utils.Rotation()
        )
    return obstacles


def dora_run(inputs):
    keys = inputs.keys()

    if (
        "depth_frame" not in keys
        or "obstacles_without_location" not in keys
        or "position" not in keys
    ):
        return {}

    obstacles = load(inputs, "obstacles_without_location")
    buffer_depth_frame = inputs["depth_frame"]
    depth_frame = np.frombuffer(
        buffer_depth_frame[: 800 * 600 * 4], dtype="uint8"
    )
    depth_frame_position = np.frombuffer(
        buffer_depth_frame[800 * 600 * 4 :], dtype="float32"
    )

    position = np.frombuffer(inputs["position"])

    position_matrix = get_projection_matrix(position)

    obstacles_with_location = get_obstacle_locations(
        obstacles, depth_frame, position, depth_frame_position
    )

    obstacles_with_prediction = get_predictions(
        obstacles_with_location, position_matrix
    )

    obstacles = dump(obstacles_with_prediction)

    return {"obstacles": obstacles}
