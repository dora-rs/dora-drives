import time
from typing import Callable

import cv2
import numpy as np
import pygame

from dora_utils import (
    LABELS,
    DoraStatus,
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
        self.lanes = []
        self.drivable_area = []
        self.last_timestamp = time.time()

    def on_input(
        self,
        input_id: str,
        value: bytes,
        _send_output: Callable[[str, bytes], None],
    ):

        if "waypoints" == input_id:
            waypoints = np.frombuffer(value)
            waypoints = waypoints.reshape((3, -1))
            waypoints = waypoints[0:2].T
            self.waypoints = waypoints
            return DoraStatus.CONTINUE

        if "obstacles_bb" == input_id:
            self.obstacles_bb = np.frombuffer(value, dtype="int32").reshape(
                (-1, 6)
            )
            return DoraStatus.CONTINUE

        if "obstacles" == input_id:
            obstacles = np.frombuffer(value, dtype="float32").reshape((-1, 5))
            self.obstacles = obstacles
            return DoraStatus.CONTINUE

        if "lanes" == input_id:
            lanes = np.frombuffer(value, dtype="int32").reshape((-1, 10, 2))
            self.lanes = lanes
            return DoraStatus.CONTINUE

        if "drivable_area" == input_id:
            drivable_area = np.frombuffer(value, dtype="int32").reshape(
                (1, -1, 2)
            )
            self.drivable_area = drivable_area
            return DoraStatus.CONTINUE

        if "image" != input_id:
            return DoraStatus.CONTINUE

        camera_frame = cv2.imdecode(
            np.frombuffer(
                value[24:],
                dtype="uint8",
            ),
            -1,
        )

        camera_frame_position = np.frombuffer(
            value[:24],
            dtype="float32",
        )

        extrinsic_matrix = get_extrinsic_matrix(
            get_projection_matrix(camera_frame_position)
        )
        resized_image = camera_frame[:, :, :3]
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
            [x, y, z, _confidence, _label] = obstacle
            location = location_to_camera_view(
                np.array([[x, y, z]]),
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
            [min_x, max_x, min_y, max_y, confidence, label] = obstacle_bb

            start = (int(min_x), int(min_y))
            end = (int(max_x), int(max_y))
            cv2.rectangle(resized_image, start, end, (0, 255, 0), 2)

            cv2.putText(
                resized_image,
                LABELS[label] + f", {confidence}%",
                (int(max_x), int(max_y)),
                font,
                0.75,
                (0, 255, 0),
                2,
                1,
            )

        for lane in self.lanes:
            cv2.polylines(resized_image, [lane], False, (0, 0, 255), 3)

        for contour in self.drivable_area:
            back = resized_image.copy()
            cv2.drawContours(back, [contour], 0, (0, 255, 0), -1)

            # blend with original image
            alpha = 0.25
            resized_image = cv2.addWeighted(
                resized_image, 1 - alpha, back, alpha, 0
            )

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
        data = cv2.flip(data, 0)

        self.last_timestamp = now
        pygame.surfarray.blit_array(gameDisplay, data)
        pygame.display.flip()

        return DoraStatus.CONTINUE
