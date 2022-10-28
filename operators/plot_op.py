import time
from typing import Callable

import cv2
import numpy as np
import zlib

from dora_utils import (
    LABELS,
    DoraStatus,
    get_extrinsic_matrix,
    get_intrinsic_matrix,
    get_projection_matrix,
    location_to_camera_view,
    to_world_coordinate,
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
        self.obstacles_bbox = []
        self.obstacles_id = []
        self.lanes = []
        self.drivable_area = []
        self.last_timestamp = time.time()
        self.position = []
        self.sensor_position = []
        self.camera_frame = []
        self.traffic_sign_bbox = []
        self.point_cloud = []

    def on_input(
        self,
        dora_input: dict,
        _send_output: Callable[[str, bytes], None],
    ):

        if "waypoints" == dora_input["id"]:
            waypoints = np.frombuffer(dora_input["data"])
            waypoints = waypoints.reshape((3, -1))
            waypoints = waypoints[0:2].T
            self.waypoints = waypoints

        elif "obstacles_bbox" == dora_input["id"]:
            self.obstacles_bbox = np.frombuffer(
                dora_input["data"], dtype="int32"
            ).reshape((-1, 6))

        elif "traffic_sign_bbox" == dora_input["id"]:
            self.traffic_sign_bbox = np.frombuffer(
                dora_input["data"], dtype="int32"
            ).reshape((-1, 6))

        elif "obstacles_id" == dora_input["id"]:
            self.obstacles_id = np.frombuffer(
                dora_input["data"], dtype="int32"
            ).reshape((-1, 7))

        elif "obstacles" == dora_input["id"]:
            obstacles = np.frombuffer(
                dora_input["data"], dtype="float32"
            ).reshape((-1, 5))
            self.obstacles = obstacles

        elif "lanes" == dora_input["id"]:
            lanes = np.frombuffer(dora_input["data"], dtype="int32").reshape(
                (-1, 30, 2)
            )
            self.lanes = lanes

        elif "drivable_area" == dora_input["id"]:
            drivable_area = np.frombuffer(
                dora_input["data"], dtype="int32"
            ).reshape((1, -1, 2))
            self.drivable_area = drivable_area

        elif "position" == dora_input["id"]:
            # Add sensor transform
            position = np.frombuffer(dora_input["data"])[:6]
            projection_matrix = get_projection_matrix(position)
            extrinsic_matrix = get_extrinsic_matrix(projection_matrix)

            sensor_transform = to_world_coordinate(
                np.array([[3.0, 0, 1.0]]), extrinsic_matrix
            )[0]
            self.position = position
            self.sensor_position = np.concatenate(
                [sensor_transform, position[3:]]
            )

        elif "image" == dora_input["id"]:
            self.camera_frame = cv2.imdecode(
                np.frombuffer(
                    dora_input["data"],
                    dtype="uint8",
                ),
                -1,
            )

        elif dora_input["id"] == "lidar_pc":
            point_cloud = np.frombuffer(
                zlib.decompress(dora_input["data"]), dtype=np.dtype("f4")
            )
            point_cloud = np.reshape(
                point_cloud, (int(point_cloud.shape[0] / 4), 4)
            )

            # To camera coordinate
            # The latest coordinate space is the unreal space.
            point_cloud = np.dot(
                point_cloud,
                np.array(
                    [[0, 0, 1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]]
                ),
            )
            point_cloud = point_cloud[np.where(point_cloud[:, 2] > 0.1)]
            self.point_cloud = point_cloud[:, :3]

        if (
            "waypoints" != dora_input["id"]
            or isinstance(self.sensor_position, list)
            or isinstance(self.camera_frame, list)
        ):
            return DoraStatus.CONTINUE

        projection_matrix = get_projection_matrix(self.position)
        extrinsic_matrix = get_extrinsic_matrix(projection_matrix)
        inv_extrinsic_matrix = np.linalg.inv(extrinsic_matrix)
        sensor_projection_matrix = get_projection_matrix(self.sensor_position)
        sensor_extrinsic_matrix = get_extrinsic_matrix(sensor_projection_matrix)
        sensor_inv_extrinsic_matrix = np.linalg.inv(sensor_extrinsic_matrix)
        resized_image = self.camera_frame[:, :, :3]
        resized_image = np.ascontiguousarray(resized_image, dtype=np.uint8)

        ## Drawing on frame

        for waypoint in self.waypoints:
            location = location_to_camera_view(
                np.append(waypoint.reshape((1, -1)), [[-1]]),
                INTRINSIC_MATRIX,
                inv_extrinsic_matrix,
            )
            if location[0] > 0 and location[1] > 0:
                cv2.circle(
                    resized_image,
                    (int(location[0]), int(location[1])),
                    3,
                    (255, 255, 255),
                    -1,
                )

        for point in self.point_cloud:

            point_world = to_world_coordinate(
                np.array([point]), extrinsic_matrix
            )
            location = location_to_camera_view(
                point_world,
                INTRINSIC_MATRIX,
                sensor_inv_extrinsic_matrix,
            )
            back = resized_image.copy()
            cv2.circle(
                back,
                (int(location[0]), int(location[1])),
                3,
                (0, 0, int(min(point[2], 255))),
                -1,
            )
            # blend with original image
            alpha = 0.25
            resized_image = cv2.addWeighted(
                resized_image, 1 - alpha, back, alpha, 0
            )

        for obstacle in self.obstacles:
            [x, y, z, _confidence, _label] = obstacle
            location = location_to_camera_view(
                np.array([[x, y, z]]),
                INTRINSIC_MATRIX,
                sensor_inv_extrinsic_matrix,
            )
            cv2.circle(
                resized_image,
                (int(location[0]), int(location[1])),
                3,
                (255, 255, 0),
                -1,
            )
            location = location_to_camera_view(
                np.array([[x, y, -1]]),
                INTRINSIC_MATRIX,
                sensor_inv_extrinsic_matrix,
            )
            cv2.circle(
                resized_image,
                (int(location[0]), int(location[1])),
                3,
                (150, 150, 0),
                -1,
            )

        for obstacle_bb in self.obstacles_bbox:
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
