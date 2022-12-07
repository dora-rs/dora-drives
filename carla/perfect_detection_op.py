import zlib
from typing import Callable

import cv2
import numpy as np
from shapely.geometry import LineString

from _dora_utils import (
    DoraStatus,
    distance_points,
    distance_vertex,
    get_extrinsic_matrix,
    get_intrinsic_matrix,
    get_projection_matrix,
    location_to_camera_view,
    to_world_coordinate,
)
from carla import Client, Vehicle, Walker

CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"

client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
client.set_timeout(30.0)  # seconds
world = client.get_world()
SEGMENTATION_THRESHOLD = 0.20
DEPTH_THRESHOLD = 5
town_name = world.get_map().name
DYNAMIC_OBSTACLE_DISTANCE_THRESHOLD = 500
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600
DEPTH_IMAGE_WIDTH = 800
DEPTH_IMAGE_HEIGHT = 600
DEPTH_IMAGE_MAX_DEPTH = 1000
DEPTH_FOV = 90
INTRINSIC_MATRIX = get_intrinsic_matrix(
    DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, DEPTH_FOV
)
SENSOR_POSITION = [3, 0, 1, 0, 0, 0]

LEFT = LineString(((0, 0), (0, IMAGE_HEIGHT)))
BOTTOM = LineString(((0, IMAGE_HEIGHT), (IMAGE_WIDTH, IMAGE_HEIGHT)))
RIGHT = LineString(((IMAGE_WIDTH, IMAGE_HEIGHT), (IMAGE_WIDTH, 0)))
TOP = LineString(((IMAGE_WIDTH, 0), (0, 0)))
CAMERA_THRESHOLDS = [LEFT, BOTTOM, RIGHT, TOP]


def threshold(p1, p2):
    points = []
    # If the points are themselves within the image, add them to the
    # set of thresholded points.
    if (
        p1[0] >= 0
        and p1[0] < IMAGE_WIDTH
        and p1[1] >= 0
        and p1[1] < IMAGE_HEIGHT
    ):
        points.append(p1)

    if (
        p2[0] >= 0
        and p2[0] < IMAGE_WIDTH
        and p2[1] >= 0
        and p2[1] < IMAGE_HEIGHT
    ):
        points.append(p2)

    # Compute the intersection of the line segment formed by p1 -- p2
    # with all the thresholds of the camera image.
    p12 = LineString((p1, p2))
    for camera_threshold in CAMERA_THRESHOLDS:
        p = p12.intersection(camera_threshold)
        if not p.is_empty:
            if p.geom_type == "Point":
                points.append((p.x, p.y))
            elif p.geom_type == "LineString":
                for coord in p.coords:
                    points.append((coord[0], coord[1]))
    return points


def to_camera_view(
    bounding_box,
    obstacle_projection: np.array,
    extrinsic_matrix,
):
    """Converts the coordinates of the bounding box for the given obstacle
    to the coordinates in the view of the camera.
    This method retrieves the extent of the bounding box, transforms them
    to coordinates relative to the bounding box origin, then converts those
    to coordinates relative to the obstacle.
    These coordinates are then considered to be in the world coordinate
    system, which is mapped into the camera view. A negative z-dora_input["data"]
    signifies that the bounding box is behind the camera plane.
    Note that this function does not cap the coordinates to be within the
    size of the camera image.
    Args:
        extrinsic_matrix: The extrinsic matrix of the camera.
    Returns:
        A list of 8 Location instances specifying the 8 corners of the
        bounding box.
    """
    # Retrieve the eight coordinates of the bounding box with respect to
    # the origin of the bounding box.

    extent = bounding_box.extent
    location = bounding_box.location

    # bounding box 8 coordinates relative to the obstacle
    bbox = np.array(
        [
            [
                location.x + extent.x,
                location.y + extent.y,
                location.z - extent.z,
            ],
            [
                location.x - extent.x,
                location.y + extent.y,
                location.z - extent.z,
            ],
            [
                location.x - extent.x,
                location.y - extent.y,
                location.z - extent.z,
            ],
            [
                location.x + extent.x,
                location.y - extent.y,
                location.z - extent.z,
            ],
            [
                location.x + extent.x,
                location.y + extent.y,
                location.z + extent.z,
            ],
            [
                location.x - extent.x,
                location.y + extent.y,
                location.z + extent.z,
            ],
            [
                location.x - extent.x,
                location.y - extent.y,
                location.z + extent.z,
            ],
            [
                location.x + extent.x,
                location.y - extent.y,
                location.z + extent.z,
            ],
        ]
    )

    # Transform the vertices with respect to the bounding box transform.
    bbox = to_world_coordinate(bbox, obstacle_projection)

    # Obstacle's transform is relative to the world. Thus, the bbox
    # contains the 3D bounding box vertices relative to the world.
    camera_coordinates = []
    for vertex in bbox:
        location_2D = location_to_camera_view(
            vertex, INTRINSIC_MATRIX, extrinsic_matrix
        )

        # Add the points to the image.
        camera_coordinates.append(location_2D)

    return np.array(camera_coordinates)


def get_bounding_box_in_camera_view(bb_coordinates) -> np.array:
    """Creates the bounding box in the view of the camera image using the
    coordinates generated with respect to the camera transform.
    Args:
        image_width (:obj:`int`): The width of the image being published by the
            camera.
        image_height (:obj:`int`): The height of the image being published by
            the camera.
    Returns:
        np.array shape (1, 4): a bounding box, or None if the bounding box
            does not fall into the view of the camera.
    """
    # Make sure that atleast 2 of the bounding box coordinates are in front.
    z_vals = [loc[2] for loc in bb_coordinates if loc[2] >= 0]
    if len(z_vals) < 2:
        return []

    # Create the thresholding line segments of the camera view.

    # Go over each of the segments of the bounding box and threshold it to
    # be inside the image.
    thresholded_points = []
    points = bb_coordinates[:, :2].astype(int)
    # Bottom plane thresholded.
    thresholded_points.extend(threshold(points[0], points[1]))
    thresholded_points.extend(threshold(points[1], points[2]))
    thresholded_points.extend(threshold(points[2], points[3]))
    thresholded_points.extend(threshold(points[3], points[0]))

    # Top plane thresholded.
    thresholded_points.extend(threshold(points[4], points[5]))
    thresholded_points.extend(threshold(points[5], points[6]))
    thresholded_points.extend(threshold(points[6], points[7]))
    thresholded_points.extend(threshold(points[7], points[4]))

    # Remaining segments thresholded.
    thresholded_points.extend(threshold(points[0], points[4]))
    thresholded_points.extend(threshold(points[1], points[5]))
    thresholded_points.extend(threshold(points[2], points[6]))
    thresholded_points.extend(threshold(points[3], points[7]))

    if len(thresholded_points) == 0:
        return []
    else:
        x = [int(x) for x, _ in thresholded_points]
        y = [int(y) for _, y in thresholded_points]
        if min(x) != max(x) and min(y) != max(y):
            return np.array([min(x), max(x), min(y), max(y)], dtype="int32")
        else:
            return []


def populate_bounding_box_2D(
    obstacle, depth_frame, segmented_frame, position
) -> np.array:
    """Populates the 2D bounding box for the obstacle.
    Heuristically uses the depth frame and segmentation frame to figure out
    if the obstacle is in view of the camera or not.
    Args:
    Returns:
       np.array: An instance representing a
        rectangle over the obstacle if the obstacle is deemed to be
        visible, None otherwise.
    """

    # Convert the bounding box of the obstacle to the camera coordinates.
    transform = obstacle.get_transform()
    obs_x = transform.location.x
    obs_y = transform.location.y
    obs_z = transform.location.z
    obs_pitch = transform.rotation.pitch
    obs_yaw = transform.rotation.yaw
    obs_roll = transform.rotation.roll
    obs_position = np.array([obs_x, obs_y, obs_z, obs_pitch, obs_yaw, obs_roll])

    bb_coordinates = to_camera_view(
        obstacle.bounding_box,
        get_projection_matrix(obs_position),
        get_extrinsic_matrix(get_projection_matrix(position + SENSOR_POSITION)),
    )

    # Threshold the bounding box to be within the camera view.
    bbox_2d = get_bounding_box_in_camera_view(bb_coordinates)

    if len(bbox_2d) == 0:
        return []
    # Crop the segmented and depth image to the given bounding box.
    cropped_image = segmented_frame[
        bbox_2d[2] : bbox_2d[3], bbox_2d[0] : bbox_2d[1]
    ]
    cropped_depth = depth_frame[
        bbox_2d[2] : bbox_2d[3], bbox_2d[0] : bbox_2d[1]
    ]

    # Get the instance of the Obstacle
    if isinstance(obstacle, Vehicle):
        segmentation_class = 10
    else:
        segmentation_class = 4

    # If the size of the bounding box is greater than 0, ensure that the
    # bounding box contains more than a threshold of pixels corresponding
    # to the required segmentation class.
    if cropped_image.size > 0:
        masked_image = np.zeros_like(cropped_image)
        masked_image[np.where(cropped_image == segmentation_class)] = 1
        if np.sum(masked_image) >= SEGMENTATION_THRESHOLD * masked_image.size:
            # The bounding box contains the required number of pixels that
            # belong to the required class. Ensure that the depth of the
            # obstacle is the depth in the image.
            masked_depth = cropped_depth[np.where(masked_image == 1)]
            mean_depth = np.mean(masked_depth) * DEPTH_IMAGE_MAX_DEPTH
            depth = distance_vertex(
                obs_position,
                position,
            )
            if abs(depth - mean_depth) <= DEPTH_THRESHOLD:
                return bbox_2d
    return []


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.position = []
        self.depth_frame = []
        self.segmented_frame = []

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        if dora_input["id"] == "depth_frame":
            depth_frame = np.frombuffer(
                zlib.decompress(dora_input["data"]),
                dtype="float32",
            )
            depth_frame = np.reshape(
                depth_frame, (DEPTH_IMAGE_HEIGHT, DEPTH_IMAGE_WIDTH)
            )

            self.depth_frame = depth_frame
            return DoraStatus.CONTINUE

        elif dora_input["id"] == "segmented_frame":
            segmented_frame = cv2.imdecode(
                np.frombuffer(
                    dora_input["data"],
                    dtype="uint8",
                ),
                -1,
            )
            self.segmented_frame = segmented_frame
            return DoraStatus.CONTINUE

        elif dora_input["id"] == "position":
            self.position = np.frombuffer(dora_input["data"], np.float32)

        if (
            len(self.position) == 0
            or len(self.depth_frame) == 0
            or len(self.segmented_frame) == 0
        ):
            return DoraStatus.CONTINUE

        actor_list = world.get_actors()
        vehicles = actor_list.filter("vehicle.*")
        pedestrians = actor_list.filter("walker.pedestrian.")
        obstacles = [vehicle for vehicle in vehicles] + [
            ped for ped in pedestrians
        ]
        outputs = []
        for obstacle in obstacles:
            # Calculate the distance of the obstacle from the vehicle, and
            # convert to camera view if it is less than
            # DYNAMIC_OBSTACLE_DISTANCE_THRESHOLD metres away.
            transform = obstacle.get_transform()
            obs_x = transform.location.x
            obs_y = transform.location.y
            obs_z = transform.location.z

            if (
                distance_points(
                    np.array([obs_x, obs_y, obs_z]), np.array(self.position[:3])
                )
                <= DYNAMIC_OBSTACLE_DISTANCE_THRESHOLD
            ):

                bbox = populate_bounding_box_2D(
                    obstacle,
                    self.depth_frame,
                    self.segmented_frame,
                    self.position,
                )
                if len(bbox) != 0:

                    # Get the instance of the Obstacle
                    if isinstance(obstacle, Vehicle):
                        segmentation_class = 2
                    else:
                        segmentation_class = 0
                    obstacle = np.append(
                        bbox,
                        np.array([100, segmentation_class], dtype="int32"),
                    )

                    outputs.append(obstacle)

        byte_array = np.array(outputs).tobytes()
        send_output(
            "bbox",
            byte_array,
            dora_input["metadata"]
            #   "traffic_lights": dump(visible_tls),
        )
        return DoraStatus.CONTINUE
