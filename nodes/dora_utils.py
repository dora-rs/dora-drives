import math
from typing import List

import numpy as np
from sklearn.metrics import pairwise_distances_argmin


def distance_vertex(left_vertix: np.array, right_vertix: np.array) -> np.array:
    return np.linalg.norm(left_vertix[:3] - right_vertix[:3])


def distance_points(left_point: np.array, right_point: np.array) -> np.array:
    return np.linalg.norm(left_point - right_point)


def closest_vertex(vertices: np.array, point: np.array) -> (int, np.array):
    assert (
        vertices.shape[1] == point.shape[1]
    ), "vertice has more coordinate than point"
    argmin_vertice = pairwise_distances_argmin(vertices, point, axis=0)[0]

    min_vertice = vertices[argmin_vertice]

    return (argmin_vertice, min_vertice)


def get_projection_matrix(position: np.array):
    """Creates a transformation matrix to convert points in the 3D world
    coordinate space with respect to the object.
    Use the transform_points function to transpose a given set of points
    with respect to the object.
    Args:
        position (np.array): [x, y, z, stir, yaw, roll]
    Returns:
        A 4x4 numpy matrix which represents the transformation matrix.
    """
    matrix = np.identity(4)
    cy = np.cos(np.radians(position[4]))
    sy = np.sin(np.radians(position[4]))
    cr = np.cos(np.radians(position[5]))
    sr = np.sin(np.radians(position[5]))
    cp = np.cos(np.radians(position[3]))
    sp = np.sin(np.radians(position[3]))
    matrix[:3, 3] = position[:3]
    matrix[0, 0] = cp * cy
    matrix[0, 1] = cy * sp * sr - sy * cr
    matrix[0, 2] = -1 * (cy * sp * cr + sy * sr)
    matrix[1, 0] = sy * cp
    matrix[1, 1] = sy * sp * sr + cy * cr
    matrix[1, 2] = cy * sr - sy * sp * cr
    matrix[2, 0] = sp
    matrix[2, 1] = -1 * (cp * sr)
    matrix[2, 2] = cp * cr
    return matrix


def to_world_coordinate(points: np.array, matrix: np.array) -> np.array:
    """Internal function to transform the points according to the
    given matrix. This function either converts the points from
    coordinate space relative to the transform to the world coordinate
    space (using self.matrix), or from world coordinate space to the
    space relative to the transform (using inv(self.matrix))

    Args:
        points: An n by 3 numpy array, where each row is the
            (x, y, z) coordinates of a point.
        matrix: The matrix of the transformation to apply.

    Returns:
        An n by 3 numpy array of transformed points.
    """
    # Needed format: [[X0,..Xn],[Y0,..Yn],[Z0,..Zn]].
    # So let's transpose the point matrix.
    points = points.transpose()

    # Add 1s row: [[X0..,Xn],[Y0..,Yn],[Z0..,Zn],[1,..1]]
    points = np.append(points, np.ones((1, points.shape[1])), axis=0)

    # Point transformation (depends on the given matrix)
    points = np.dot(matrix, points)

    # Get all but the last row in array form.
    points = np.asarray(points[0:3].transpose()).astype(np.float16)

    return points


def get_extrinsic_matrix(transform):
    """Converts a Transform from the camera coordinate space to the
    Unreal coordinate space.
    The camera space is defined as:
        +x to right, +y to down, +z into the screen.
    The unreal coordinate space is defined as:
        +x into the screen, +y to right, +z to up.
    Args:
        transform (:py:class:`~pylot.utils.Transform`): The transform to
            convert to Unreal coordinate space.
    Returns:
        :py:class:`~pylot.utils.Transform`: The given transform after
            transforming to the Unreal coordinate space.
    """

    to_unreal_transform = np.array(
        [[0, 0, 1, 0], [1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 0, 1]]
    )
    return transform * to_unreal_transform


def get_intrinsic_matrix(width: int, height: int, fov: float):
    """Creates the intrinsic matrix for a camera with the given
    parameters.
    Args:
        width (int): The width of the image returned by the camera.
        height (int): The height of the image returned by the camera.
        fov (float): The field-of-view of the camera.
    Returns:
        :py:class:`numpy.ndarray`: A 3x3 intrinsic matrix of the camera.
    """

    k = np.identity(3)
    # We use width - 1 and height - 1 to find the center column and row
    # of the image, because the images are indexed from 0.

    # Center column of the image.
    k[0, 2] = (width - 1) / 2.0
    # Center row of the image.
    k[1, 2] = (height - 1) / 2.0
    # Focal length.
    k[0, 0] = k[1, 1] = (width - 1) / (2.0 * np.tan(fov * np.pi / 360.0))
    return k


def get_angle(left, right) -> float:
    """Computes the angle between the vector and another vector
    in radians."""
    [left_x, left_y] = left
    [right_x, right_y] = right

    angle = math.atan2(left_y, left_x) - math.atan2(right_y, right_x)
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle
