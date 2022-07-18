from collections import deque

import cv2
import numpy as np
from pylot.utils import Location, Rotation, Transform, Vector3D
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon


class Lane(object):
    """Stores information about a lane.

    Args:
        id (:obj:`int`): The id of the lane (0 for ego lane, negative for
            left lanes, and positive for right lanes).
        left_markings: List of transforms.
        right_markings: List of transforms.
    """

    def __init__(self, id: int, left_markings, right_markings):
        self.id = id
        self.left_markings = left_markings
        self.right_markings = right_markings
        self._lane_polygon = None
        self._color_map = [
            (255, 0, 0),
            (0, 255, 0),
            (0, 0, 255),
            (125, 125, 0),
            (0, 125, 125),
            (125, 0, 125),
            (50, 100, 50),
            (100, 50, 100),
        ]

    def get_closest_lane_waypoint(self, location):
        if self.is_on_lane(location):
            return Transform(location, Rotation())
        closest_transform = None
        min_dist = np.infty
        for transform in self.left_markings:
            dist = transform.location.distance(location)
            if dist < min_dist:
                min_dist = dist
                closest_transform = transform
        for transform in self.right_markings:
            dist = transform.location.distance(location)
            if dist < min_dist:
                min_dist = dist
                closest_transform = transform
        return closest_transform

    def get_lane_center_transforms(self):
        if len(self.left_markings) < len(self.right_markings):
            anchor_markings = self.left_markings
            other_markings = self.right_markings
        else:
            anchor_markings = self.right_markings
            other_markings = self.left_markings
        index_other = 0
        center_markings = deque([])
        for transform in anchor_markings:
            dist = transform.location.distance(
                other_markings[index_other].location
            )
            while index_other + 1 < len(
                other_markings
            ) and dist > transform.location.distance(
                other_markings[index_other + 1].location
            ):
                index_other += 1
                dist = transform.location.distance(
                    other_markings[index_other].location
                )
            if index_other < len(other_markings):
                other_loc = other_markings[index_other].location
                center_location = Location(
                    (transform.location.x + other_loc.x) / 2.0,
                    (transform.location.y + other_loc.y) / 2.0,
                    (transform.location.z + other_loc.z) / 2.0,
                )
                center_markings.append(Transform(center_location, Rotation()))
        return center_markings

    def is_on_lane(self, location):
        # We only create the lane polygon if it is necessary.
        if not self._lane_polygon:
            self._create_lane_polygon()
        return self._lane_polygon.contains(Point(location.x, location.y))

    def _create_lane_polygon(self):
        points = [(0, self.left_markings[0].y)]
        for transform in self.left_markings:
            points.append((transform.location.x, transform.location.y))
        for transform in reversed(self.right_markings):
            points.append((transform.location.x, transform.location.y))
        points.append((0, self.right_markings[0].y))
        self._lane_polygon = Polygon(points)

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "Lane(id: {}, {})".format(
            self.id, zip(self.left_markings, self.right_markings)
        )
