import time
from typing import Callable
from enum import Enum

import cv2
import numpy as np

font = cv2.FONT_HERSHEY_SIMPLEX
bottomLeftCornerOfText = (10, 500)
fontScale = 1
fontColor = (255, 255, 255)
thickness = 1
lineType = 2


class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.obstacles_bb = []
        self.obstacles_id = []
        self.lanes = []
        self.drivable_area = []
        self.last_timestamp = time.time()
        self.camera_frame = []

    def on_input(
        self,
        input_id: str,
        value: bytes,
        _send_output: Callable[[str, bytes], None],
    ):

        if "obstacles_bb" == input_id:
            self.obstacles_bb = np.frombuffer(value, dtype="int32").reshape(
                (-1, 6)
            )

        elif "obstacles_id" == input_id:
            self.obstacles_id = np.frombuffer(value, dtype="int32").reshape(
                (-1, 7)
            )

        elif "lanes" == input_id:
            lanes = np.frombuffer(value, dtype="int32").reshape((-1, 10, 2))
            self.lanes = lanes

        elif "drivable_area" == input_id:
            drivable_area = np.frombuffer(value, dtype="int32").reshape(
                (1, -1, 2)
            )
            self.drivable_area = drivable_area

        elif "image" == input_id:
            self.camera_frame = cv2.imdecode(
                np.frombuffer(
                    value,
                    dtype="uint8",
                ),
                -1,
            )

        if "image" != input_id:
            return DoraStatus.CONTINUE

        resized_image = self.camera_frame[:, :, :3]
        resized_image = np.ascontiguousarray(resized_image, dtype=np.uint8)

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
            if len(contour) != 0:
                cv2.drawContours(back, [contour], 0, (0, 255, 0), -1)

                # blend with original image
                alpha = 0.25
                resized_image = cv2.addWeighted(
                    resized_image, 1 - alpha, back, alpha, 0
                )

        cv2.imshow("image", resized_image)
        cv2.waitKey(1)


        return DoraStatus.CONTINUE
