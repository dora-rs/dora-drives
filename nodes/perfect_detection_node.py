import numpy as np
from carla import Client, Vehicle, Walker
from shapely.geometry import LineString

from dora_utils import (
    get_extrinsic_matrix,
    get_intrinsic_matrix,
    get_projection_matrix,
    to_world_coordinate,
)

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
DEPTH_FOV = 90
INTRINSIC_MATRIX = get_intrinsic_matrix(
    DEPTH_IMAGE_WIDTH, DEPTH_IMAGE_HEIGHT, DEPTH_FOV
)


def threshold(p1, p2, camera_thresholds):
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
    for camera_threshold in camera_thresholds:
        p = p12.intersection(camera_threshold)
        if not p.is_empty:
            if p.geom_type == "Point":
                points.append((p.x, p.y))
            elif p.geom_type == "LineString":
                for coord in p.coords:
                    points.append((coord[0], coord[1]))
    return points


def location_to_camera_view(location: np.array, extrinsic_matrix):
    """Converts the given 3D vector to the view of the camera using
    the extrinsic and the intrinsic matrix.
    Args:
        extrinsic_matrix: The extrinsic matrix of the camera.
    Returns:
        :py:class:`.Vector3D`: An instance with the coordinates converted
        to the camera view.
    """
    position_vector = location.T
    position_vector += [[1.0]]

    # Transform the points to the camera in 3D.
    transformed_3D_pos = np.dot(
        np.linalg.inv(extrinsic_matrix), position_vector
    )

    # Transform the points to 2D.
    position_2D = np.dot(INTRINSIC_MATRIX, transformed_3D_pos[:3])

    # Normalize the 2D points.
    location_2D = np.array(
        [
            float(position_2D[0] / position_2D[2]),
            float(position_2D[1] / position_2D[2]),
            float(position_2D[2]),
        ]
    )
    return location_2D


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
    system, which is mapped into the camera view. A negative z-value
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
        location_2D = location_to_camera_view(vertex, extrinsic_matrix)

        # Add the points to the image.
        camera_coordinates.append(location_2D)

    return camera_coordinates


def get_bounding_box_in_camera_view(
    bb_coordinates, image_width, image_height
) -> np.array:
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
    z_vals = [loc.z for loc in bb_coordinates if loc.z >= 0]
    if len(z_vals) < 2:
        return None

    # Create the thresholding line segments of the camera view.

    left = LineString(((0, 0), (0, image_height)))
    bottom = LineString(((0, image_height), (image_width, image_height)))
    right = LineString(((image_width, image_height), (image_width, 0)))
    top = LineString(((image_width, 0), (0, 0)))
    camera_thresholds = [left, bottom, right, top]

    # Go over each of the segments of the bounding box and threshold it to
    # be inside the image.
    thresholded_points = []
    points = [(int(loc.x), int(loc.y)) for loc in bb_coordinates]
    # Bottom plane thresholded.
    thresholded_points.extend(
        threshold(points[0], points[1], camera_thresholds)
    )
    thresholded_points.extend(
        threshold(points[1], points[2], camera_thresholds)
    )
    thresholded_points.extend(
        threshold(points[2], points[3], camera_thresholds)
    )
    thresholded_points.extend(
        threshold(points[3], points[0], camera_thresholds)
    )

    # Top plane thresholded.
    thresholded_points.extend(
        threshold(points[4], points[5], camera_thresholds)
    )
    thresholded_points.extend(
        threshold(points[5], points[6], camera_thresholds)
    )
    thresholded_points.extend(
        threshold(points[6], points[7], camera_thresholds)
    )
    thresholded_points.extend(
        threshold(points[7], points[4], camera_thresholds)
    )

    # Remaining segments thresholded.
    thresholded_points.extend(
        threshold(points[0], points[4], camera_thresholds)
    )
    thresholded_points.extend(
        threshold(points[1], points[5], camera_thresholds)
    )
    thresholded_points.extend(
        threshold(points[2], points[6], camera_thresholds)
    )
    thresholded_points.extend(
        threshold(points[3], points[7], camera_thresholds)
    )

    if len(thresholded_points) == 0:
        return None
    else:
        x = [int(x) for x, _ in thresholded_points]
        y = [int(y) for _, y in thresholded_points]
        if min(x) < max(x) and min(y) < max(y):
            return np.array([min(x), max(x), min(y), max(y)], dtype="float32")
        else:
            return None


def populate_bounding_box_2D(
    obstacle, depth_frame, segmented_frame, depth_frame_position
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
    bb_coordinates = to_camera_view(
        obstacle.bounding_box,
        get_projection_matrix(obstacle.transform),
        get_extrinsic_matrix(get_projection_matrix(depth_frame_position)),
    )

    # Threshold the bounding box to be within the camera view.
    bbox_2d = get_bounding_box_in_camera_view(
        bb_coordinates,
        DEPTH_IMAGE_WIDTH,
        DEPTH_IMAGE_HEIGHT,
    )

    if not bbox_2d:
        return None
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
        seg_threshold = SEGMENTATION_THRESHOLD * masked_image.size
        if np.sum(masked_image) >= seg_threshold:
            # The bounding box contains the required number of pixels that
            # belong to the required class. Ensure that the depth of the
            # obstacle is the depth in the image.
            masked_depth = cropped_depth[np.where(masked_image == 1)]
            mean_depth = np.mean(masked_depth) * 1000
            transform = obstacle.get_transform()
            obs_x = transform.location.x
            obs_y = transform.location.y
            obs_z = transform.location.z
            depth = np.linalg.norm(
                [obs_x, obs_y, obs_z] - depth_frame_position[:3]
            )
            if abs(depth - mean_depth) <= DEPTH_THRESHOLD:
                return bbox_2d
    return None


def dora_run(inputs):
    keys = inputs.keys()

    if (
        "position" not in keys
        or "segmented_frame" not in keys
        or "depth_frame" not in keys
    ):
        return {}

    position = np.frombuffer(inputs["position"])

    buffer_depth_frame = inputs["depth_frame"]
    buffer_segmented_frame = inputs["segmented_frame"]
    depth_frame = np.frombuffer(
        buffer_depth_frame[: 800 * 600 * 4], dtype="uint8"
    )
    segmented_frame = np.frombuffer(
        buffer_segmented_frame[: 800 * 600 * 4], dtype="uint8"
    )
    depth_frame_position = np.frombuffer(
        buffer_depth_frame[800 * 600 * 4 :], dtype="float32"
    )

    actor_list = world.get_actors()
    obstacles = actor_list.filter("walker.pedestrian.*|vehicle.*")

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
            np.linalg.norm([obs_x, obs_y, obs_z] - position[:3])
            <= DYNAMIC_OBSTACLE_DISTANCE_THRESHOLD
        ):

            bbox = populate_bounding_box_2D(
                obstacle, depth_frame, segmented_frame, depth_frame_position
            )
            if bbox:

                # Get the instance of the Obstacle
                if isinstance(obstacle, Vehicle):
                    segmentation_class = 10
                else:
                    segmentation_class = 4

                obstacle_bytes = (
                    bbox.tobytes()
                    + depth_frame_position.tobytes()
                    + np.array(
                        [1.0, segmentation_class], dtype="float32"
                    ).tobytes()
                )

                outputs.append(obstacle_bytes)

    byte_array = b"".join(outputs)

    return {
        "obstacles_without_location": byte_array,
        #   "traffic_lights": dump(visible_tls),
    }
