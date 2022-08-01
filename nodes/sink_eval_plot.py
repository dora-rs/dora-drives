import logging
import threading
import time

import cv2
import numpy as np
import pygame

from dora_utils import (
    get_extrinsic_matrix,
    get_intrinsic_matrix,
    get_projection_matrix,
    location_to_camera_view,
)

mutex = threading.Lock()
pygame.init()

goal_location = [234, 59, 39]


class Flags(object):
    pass


FLAGS = Flags()
FLAGS.tracking_num_steps = 10
FLAGS.step_size = 1
FLAGS.max_iterations = 10
FLAGS.end_dist_threshold = 1
FLAGS.obstacle_clearance_rrt = 1
FLAGS.lane_width = 1
FLAGS.planning_type = "waypoints"
FLAGS.target_speed = 10
FLAGS.max_speed = 10
FLAGS.max_accel = 10
FLAGS.max_step_hybrid_astar = 10
FLAGS.step_size_hybrid_astar = 10
FLAGS.max_iterations_hybrid_astar = 10
FLAGS.completion_threshold = 10
FLAGS.angle_completion_threshold = 10
FLAGS.rad_step = 10
FLAGS.rad_upper_range = 10
FLAGS.rad_lower_range = 10
FLAGS.max_curvature = 10
FLAGS.num_waypoints_ahead = 10
FLAGS.obstacle_clearance_hybrid_astar = 10
FLAGS.lane_width_hybrid_astar = 10
FLAGS.radius = 10
FLAGS.car_length = 10
FLAGS.car_width = 10
FLAGS.static_obstacle_distance_threshold = 1000


logger = logging.Logger("")


CAMERA_WIDTH = 800
CAMERA_HEIGHT = 600

gameDisplay = pygame.display.set_mode(
    (CAMERA_WIDTH, CAMERA_HEIGHT),
    pygame.HWSURFACE | pygame.DOUBLEBUF,
)
font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10, 500)
fontScale = 1
fontColor = (255, 255, 255)
thickness = 1
lineType = 2

counter = time.time()

DEPTH_IMAGE_WIDTH = 800
DEPTH_IMAGE_HEIGHT = 600
DEPTH_FOV = 90
INTRINSIC_MATRIX = get_intrinsic_matrix(
    DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, DEPTH_FOV
)

old_waypoints = None
old_obstacles = None
old_obstacles_bb = None


def dora_run(inputs):

    keys = inputs.keys()
    if "image" not in keys:
        return {}

    if "position" not in keys:
        return {}
    global mutex
    mutex.acquire()
    global old_waypoints
    global old_obstacles
    global old_obstacles_bb

    buffer_camera_frame = inputs["image"]
    camera_frame = np.frombuffer(
        buffer_camera_frame[: DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT * 4],
        dtype="uint8",
    )

    camera_frame_position = np.frombuffer(
        buffer_camera_frame[DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT * 4 :],
        dtype="float32",
    )

    # [x, y, z, pitch, yaw, roll, current_speed] = position
    extrinsic_matrix = get_extrinsic_matrix(
        get_projection_matrix(camera_frame_position)
    )

    if "obstacles_bb" in keys:
        obstacles_bb = inputs["obstacles_bb"]
        obstacles_bb = obstacles_bb.split(b"\n")
    elif old_obstacles_bb is not None:
        obstacles_bb = old_obstacles_bb
    else:
        obstacles_bb = []

    if "obstacles" in keys:
        obstacles = inputs["obstacles"]
        obstacles = obstacles.split(b"\n")
    elif old_obstacles is not None:
        obstacles = old_obstacles
    else:
        obstacles = []

    if "waypoints" in keys:
        waypoints = np.frombuffer(inputs["waypoints"])
        waypoints = waypoints.reshape((3, -1))
        waypoints = waypoints[0:2].T

    elif old_waypoints is not None:
        waypoints = old_waypoints
    else:
        waypoints = None

    # obstacle_prediction.draw_trajectory_on_frame(image)
    if len(camera_frame) == DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT * 4:
        resized_image = np.reshape(
            camera_frame, (CAMERA_HEIGHT, CAMERA_WIDTH, 4)
        )
        resized_image = resized_image[:, :, :3]
        resized_image = np.ascontiguousarray(resized_image, dtype=np.uint8)
    else:
        resized_image = np.reshape(
            camera_frame, (CAMERA_HEIGHT, CAMERA_WIDTH, 3)
        )

    # Draw Waypoints
    if waypoints is not None:
        for waypoint in waypoints:
            location = location_to_camera_view(
                np.append(waypoint.reshape((1, -1)), [[0]]),
                INTRINSIC_MATRIX,
                extrinsic_matrix,
            )
            if location[0] > 0 and location[1] > 0:
                cv2.circle(
                    resized_image,
                    (int(location[0]), int(location[1])),
                    3,
                    (255, 255, 255),
                    -1,
                )

    for obstacle in obstacles:
        if len(obstacle) % 12 == 0:
            obstacle_buffer = np.frombuffer(obstacle, dtype="float32")

            obstacle_position = np.reshape(obstacle_buffer, (-1, 3))
            for point in obstacle_position:
                location = location_to_camera_view(
                    np.append(location[:2].reshape((1, -1)), [[0]]),
                    INTRINSIC_MATRIX,
                    extrinsic_matrix,
                )
                cv2.circle(
                    resized_image,
                    (int(location[0]), int(location[1])),
                    3,
                    (255, 255, 0),
                    -1,
                )
        # else:
        # obstacle_buffer = np.frombuffer(obstacle)
        # print(obstacle_buffer)

    for obstacle_bb in obstacles_bb:
        if len(obstacle_bb) > 16:
            obstacle_bb_buffer = np.frombuffer(obstacle_bb[:16], dtype="int32")
            if len(obstacle_bb_buffer) != 0:
                [min_x, max_x, min_y, max_y] = obstacle_bb_buffer

                start = (int(min_x), int(min_y))
                end = (int(max_x), int(max_y))
                cv2.rectangle(resized_image, start, end, (0, 255, 0), 2)

    global counter
    now = time.time()
    cv2.putText(
        resized_image,
        f"Hertz {1 / (now - counter):.2f}",
        bottomLeftCornerOfText,
        font,
        fontScale,
        fontColor,
        thickness,
        lineType,
    )
    data = resized_image[:, :, (2, 1, 0)]
    data = np.rot90(data)

    counter = now
    pygame.surfarray.blit_array(gameDisplay, data)
    pygame.display.flip()
    old_obstacles = obstacles
    old_waypoints = waypoints
    mutex.release()

    return {}
