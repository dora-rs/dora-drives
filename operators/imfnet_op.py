import math
import time
import zlib
from typing import Callable

import cv2
import numpy as np
import open3d as o3d
import torch
from dora import DoraStatus
from imfnet import (
    extract_features,
    get_model,
    make_open3d_feature_from_numpy,
    make_open3d_point_cloud,
    process_image,
)

VOXEL_SIZE = 2.5


def run_ransac(xyz0, xyz1, feat0, feat1, voxel_size, ransac_n=4):
    distance_threshold = voxel_size * 1.5
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source=xyz0,
        target=xyz1,
        source_feature=feat0,
        target_feature=feat1,
        max_correspondence_distance=distance_threshold,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(
            False
        ),
        ransac_n=ransac_n,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9
            ),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold
            ),
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(
            1_000_000, 0.999
        ),
        mutual_filter=False,
    )
    return result_ransac


class Operator:
    """
    Infering object from images
    """

    def __init__(self):
        self.model, self.config = get_model()
        self.frame = []
        self.point_cloud = []
        self.previous_pc_down = []
        self.previous_features = []

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        """Handle image
        Args:
            dora_input["id"](str): Id of the input declared in the yaml configuration
            dora_input["data"] (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """

        if dora_input["id"] == "lidar_pc":
            point_cloud = np.frombuffer(
                zlib.decompress(dora_input["data"]), dtype=np.dtype("f4")
            )
            point_cloud = np.reshape(
                point_cloud, (int(point_cloud.shape[0] / 4), 4)
            )

            # Default Unreal coordinate is:
            # +x is forward, +y to the right, and +z up
            # To velodyne coordinate first
            # +x into the screen, +y to the left, and +z up.
            # The latest coordinate space is the unreal space.

            point_cloud = np.dot(
                point_cloud,
                np.array(
                    [[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
                ),
            )

            point_cloud = point_cloud[:, :3]
            self.point_cloud = point_cloud

        if dora_input["id"] == "image":
            frame = cv2.imdecode(
                np.frombuffer(
                    dora_input["data"],
                    dtype="uint8",
                ),
                -1,
            )
            # if len(self.frame) != 0:
            # cv2.imwrite("previous_image.jpg", self.frame)
            # cv2.imwrite("current_image.jpg", frame)

            frame = frame[:, :, :3]

            if (
                frame.shape[0] != self.config.image_H
                or frame.shape[1] != self.config.image_W
            ):
                image = process_image(
                    image=frame,
                    aim_H=self.config.image_H,
                    aim_W=self.config.image_W,
                )

            image = np.transpose(image, axes=(2, 0, 1))
            self.frame = np.expand_dims(image, axis=0)
            return DoraStatus.CONTINUE

        if len(self.frame) == 0:
            return DoraStatus.CONTINUE
        # Extract features
        current_pc_down, current_features = extract_features(
            self.model,
            xyz=self.point_cloud,
            rgb=None,
            normal=None,
            voxel_size=VOXEL_SIZE,
            device=torch.device("cuda"),
            skip_check=True,
            image=self.frame,
        )

        current_pc_down = make_open3d_point_cloud(current_pc_down)
        current_features = current_features.cpu().detach().numpy()
        current_features = make_open3d_feature_from_numpy(current_features)

        if isinstance(self.previous_features, list):
            self.previous_pc_down = current_pc_down
            self.previous_features = current_features
            return DoraStatus.CONTINUE

        timestamp = time.time()
        print("running ransac")
        result = run_ransac(
            self.previous_pc_down,
            current_pc_down,
            self.previous_features,
            current_features,
            VOXEL_SIZE,
        )
        T = result.transformation

        if result.fitness < 0.6:
            print(f"fitness: {result.fitness}")
            print("could not fit IMFnet")
            self.previous_pc_down = current_pc_down
            self.previous_features = current_features
            return DoraStatus.CONTINUE

        if result.fitness == 1.0:
            print("car did not move")
            self.previous_pc_down = current_pc_down
            self.previous_features = current_features
            return DoraStatus.CONTINUE

        print(f"elapsed: {time.time() - timestamp}")

        self.previous_pc_down.paint_uniform_color([1, 0.706, 0])
        current_pc_down.paint_uniform_color([0, 0.651, 0.929])
        o3d.visualization.draw_geometries(
            [self.previous_pc_down, current_pc_down]
        )
        # o3d.io.write_point_cloud("previous_pc.pcd", self.previous_pc_down)
        # o3d.io.write_point_cloud("current_pc.pcd", current_pc_down)
        self.previous_pc_down.transform(T)
        o3d.visualization.draw_geometries(
            [self.previous_pc_down, current_pc_down]
        )

        # Velodyne coordinate is
        # +x into the screen, +y to the left, and +z up.
        # Camera coordinate is
        # +x to right, +y to down, +z into the screen.
        pitch_r = math.atan2(T[2, 1], T[2, 2])
        yaw_r = math.atan2(T[1, 0], T[0, 0])
        roll_r = -math.atan2(-T[2, 0], math.sqrt(T[2, 1] ** 2 + T[2, 2] ** 2))
        x, y, z = -T[0, 3], T[1, 3], T[2, 3]

        relative_position = np.array(
            [
                x,
                y,
                z,
                math.degrees(pitch_r),
                math.degrees(yaw_r),
                math.degrees(roll_r),
            ],
            dtype=np.float32,
        )
        print(f"Infered: {relative_position}")

        print(f"fitness: {result.fitness}")
        # send_output("relative_position", position.tobytes())

        self.previous_pc_down = current_pc_down
        self.previous_features = current_features

        return DoraStatus.CONTINUE
