import time
from typing import Callable

import cv2
import numpy as np

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
SENSOR_POSITION = np.array([3, 0, 1, 0, 0, 0])
INTRINSIC_MATRIX = get_intrinsic_matrix(
    DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, DEPTH_FOV
)
writer = cv2.VideoWriter(
    "output.avi",
    cv2.VideoWriter_fourcc(*"MJPG"),
    30,
    (CAMERA_WIDTH, CAMERA_HEIGHT),
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
        self.obstacles_id = []
        self.lanes = []
        self.drivable_area = []
        self.last_timestamp = time.time()
        self.position = []
        self.camera_frame = []

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

        elif "obstacles_bb" == input_id:
            self.obstacles_bb = np.frombuffer(value, dtype="int32").reshape(
                (-1, 6)
            )

        elif "obstacles_id" == input_id:
            self.obstacles_id = np.frombuffer(value, dtype="int32").reshape(
                (-1, 7)
            )

        elif "obstacles" == input_id:
            obstacles = np.frombuffer(value, dtype="float32").reshape((-1, 5))
            self.obstacles = obstacles

        elif "lanes" == input_id:
            lanes = np.frombuffer(value, dtype="int32").reshape((-1, 10, 2))
            self.lanes = lanes

        elif "drivable_area" == input_id:
            drivable_area = np.frombuffer(value, dtype="int32").reshape(
                (1, -1, 2)
            )
            self.drivable_area = drivable_area

        elif "position" == input_id:
            # Add sensor transform
            self.position = np.frombuffer(value)[:6] + SENSOR_POSITION

        elif "image" == input_id:
            self.camera_frame = cv2.imdecode(
                np.frombuffer(
                    value,
                    dtype="uint8",
                ),
                -1,
            )

        if "image" != input_id or isinstance(self.position, list):
            return DoraStatus.CONTINUE

        extrinsic_matrix = get_extrinsic_matrix(
            get_projection_matrix(self.position)
        )
        resized_image = self.camera_frame[:, :, :3]
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

        for obstacle_id in self.obstacles_id:
            [
                min_x,
                max_x,
                min_y,
                max_y,
                track_id,
                confidence,
                label,
            ] = obstacle_id
            start = (int(min_x), int(min_y))
            end = (int(max_x), int(max_y))
            # cv2.rectangle(resized_image, start, end, (0, 255, 0), 2)

            cv2.putText(
                resized_image,
                f"#{track_id}",
                (int(max_x), int(max_y + 20)),
                font,
                0.75,
                (255, 140, 0),
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
        # cv2.putText(
        # resized_image,
        # f"Hertz {1 / (now - self.last_timestamp):.2f}",
        # bottomLeftCornerOfText,
        # font,
        # fontScale,
        # fontColor,
        # thickness,
        # lineType,
        # )

        self.last_timestamp = now
        writer.write(resized_image)
        cv2.imshow("image", resized_image)
        cv2.waitKey(1)

        return DoraStatus.CONTINUE
