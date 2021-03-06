import time
from typing import Callable

import cv2
import numpy as np
import pygame

from dora_utils import (
    get_extrinsic_matrix,
    get_intrinsic_matrix,
    get_projection_matrix,
    location_to_camera_view,
)

CAMERA_WIDTH = 800
CAMERA_HEIGHT = 600
DEPTH_IMAGE_WIDTH = 800
DEPTH_IMAGE_HEIGHT = 600
DEPTH_FOV = 90
INTRINSIC_MATRIX = get_intrinsic_matrix(
    DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, DEPTH_FOV
)

pygame.init()
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


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.waypoints = []
        self.obstacles = []
        self.obstacles_bb = []
        self.last_timestamp = time.time()

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ):

        if "waypoints" == input_id:
            waypoints = np.frombuffer(value)
            waypoints = waypoints.reshape((3, -1))
            waypoints = waypoints[0:2].T
            self.waypoints = waypoints
            return {}

        if "obstacles_bb" == input_id:
            obstacles_bb = value.split(b"\n")
            self.obstacles_bb = obstacles_bb
            return {}

        if "obstacles" == input_id:
            obstacles = value.split(b"\n")
            self.obstacles = obstacles
            return {}

        if "image" != input_id:
            return {}

        camera_frame = np.frombuffer(
            value[: DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT * 4],
            dtype="uint8",
        )

        camera_frame_position = np.frombuffer(
            value[DEPTH_IMAGE_WIDTH * DEPTH_IMAGE_HEIGHT * 4 :],
            dtype="float32",
        )

        extrinsic_matrix = get_extrinsic_matrix(
            get_projection_matrix(camera_frame_position)
        )
        resized_image = np.reshape(
            camera_frame, (CAMERA_HEIGHT, CAMERA_WIDTH, 4)
        )
        resized_image = resized_image[:, :, :3]
        resized_image = np.ascontiguousarray(resized_image, dtype=np.uint8)

        ## Drawing on frame

        for waypoint in self.waypoints:
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

        for obstacle in self.obstacles:
            if len(obstacle) % 12 == 0:
                obstacle_buffer = np.frombuffer(obstacle, dtype="float32")

                obstacle_position = np.reshape(obstacle_buffer, (-1, 3))
                for location in obstacle_position:
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

        for obstacle_bb in self.obstacles_bb:
            if len(obstacle_bb) > 16:
                obstacle_bb_buffer = np.frombuffer(
                    obstacle_bb[:16], dtype="int32"
                )
                if len(obstacle_bb_buffer) != 0:
                    [min_x, max_x, min_y, max_y] = obstacle_bb_buffer

                    start = (int(min_x), int(min_y))
                    end = (int(max_x), int(max_y))
                    cv2.rectangle(resized_image, start, end, (0, 255, 0), 2)

        now = time.time()
        cv2.putText(
            resized_image,
            f"Hertz {1 / (now - self.last_timestamp):.2f}",
            bottomLeftCornerOfText,
            font,
            fontScale,
            fontColor,
            thickness,
            lineType,
        )
        data = resized_image[:, :, (2, 1, 0)]
        data = np.rot90(data)

        self.last_timestamp = now
        pygame.surfarray.blit_array(gameDisplay, data)
        pygame.display.flip()

        return {}
