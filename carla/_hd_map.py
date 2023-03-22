"""Wrapper module for interacting with the CARLA HD map.

This module implements HDMap class which offers utility methods for interacting
with the CARLA HD map.
"""


import numpy as np

# Import Planner from CARLA codebase
from agents.navigation.global_route_planner import GlobalRoutePlanner

from carla import LaneType, Location


class HDMap(object):
    """Wrapper class around the CARLA map.

    All Pylot methods should strive to use this class instead of directly
    accessing a CARLA map. This will make it easier to extend the probject
    with support for other types of HD maps in the future.

    Attributes:
        _map: An instance of a CARLA map.
        _grp: An instance of a CARLA global route planner (uses A*).
    """

    def __init__(self, simulator_map, _log_file=None):
        self._map = simulator_map
        # Setup global planner.
        self._grp = GlobalRoutePlanner(
            self._map, 1.0
        )  # Distance between waypoints

    # self._grp.setup()

    def is_intersection(self, location: np.array) -> bool:
        """Checks if a location is in an intersection.

        Args:
            location (:py:class:`~pylot.utils.Location`): Location in world
                coordinates.

        Returns:
            bool: True if the location is in an intersection.
        """
        waypoint = self._get_waypoint(location)
        if not waypoint:
            # The map didn't return a waypoint because the location not within
            # mapped location.
            return False
        else:
            return self.__is_intersection(waypoint)

    def __is_intersection(self, waypoint) -> bool:
        if waypoint.is_junction:
            return True
        if hasattr(waypoint, "is_intersection"):
            return waypoint.is_intersection
        return False

    def are_on_same_lane(
        self, location1: np.array, location2: np.array
    ) -> bool:
        """Checks if two locations are on the same lane.

        Args:
            location1 (:py:class:`~pylot.utils.Location`): Location in world
                coordinates.
            location2 (:py:class:`~pylot.utils.Location`): Location in world
                coordinates.

        Returns:
            bool: True if the two locations are on the same lane.
        """
        waypoint1 = self._get_waypoint(location1, lane_type=LaneType.Driving)
        if not waypoint1:
            # First location is not on a drivable lane.
            return False
        waypoint2 = self._get_waypoint(location2, lane_type=LaneType.Driving)
        if not waypoint2:
            # Second location is not on a drivable lane.
            return False
        if waypoint1.road_id == waypoint2.road_id:
            return waypoint1.lane_id == waypoint2.lane_id
        else:
            # Return False if we're in intersection and the other
            # obstacle isn't.
            if self.__is_intersection(waypoint1) and not self.__is_intersection(
                waypoint2
            ):
                return False
            if waypoint2.lane_type == LaneType.Driving:
                # This may return True when the lane is different, but in
                # with a different road_id.
                # TODO(ionel): Figure out how lane id map across road id.
                return True
        return False

    def distance_to_intersection(
        self, location: np.array, max_distance_to_check: float = 30
    ):
        """Computes the distance (in meters) from location to an intersection.

        The method starts from location, moves forward until it reaches an
        intersection or exceeds max_distance_to_check.

        Args:
            location (:py:class:`~pylot.utils.Location`): The starting location
                in world coordinates.
            max_distance_to_check (:obj:`int`): Max distance to move forward
                 (in meters).

        Returns:
            :obj:`int`: The distance in meters, or None if there is no
            intersection within max_distance_to_check.
        """
        waypoint = self._get_waypoint(location)
        if not waypoint:
            return None
        # We're already in an intersection.
        if self.__is_intersection(waypoint):
            return 0
        for i in range(1, max_distance_to_check + 1):
            waypoints = waypoint.next(1)
            if not waypoints or len(waypoints) == 0:
                return None
            for w in waypoints:
                if self.__is_intersection(w):
                    return i
            waypoint = waypoints[0]
        return None

    def compute_waypoints(
        self, source_loc: np.array, destination_loc: np.array
    ):
        """Computes waypoints between two locations.

        Assumes that the ego vehicle has the same orientation as the lane on
        whch it is on.

        Args:
        source_loc (:py:class:`~pylot.utils.Location`): Source location in
        world coordinates.
        destination_loc (:py:class:`~pylot.utils.Location`): Destination
        location in world coordinates.

        Returns:
        list(:py:class:`~pylot.utils.Transform`): List of waypoint
        transforms.
        """
        start_waypoint = self._get_waypoint(
            source_loc, project_to_road=True, lane_type=LaneType.Driving
        )
        end_waypoint = self._get_waypoint(
            destination_loc, project_to_road=True, lane_type=LaneType.Driving
        )
        assert start_waypoint and end_waypoint, "Map could not find waypoints"
        route = self._grp.trace_route(
            start_waypoint.transform.location, end_waypoint.transform.location
        )
        return np.array(
            [
                [
                    waypoint[0].transform.location.x,
                    waypoint[0].transform.location.y,
                ]
                for waypoint in route
            ]
        )

    def _get_waypoint(
        self,
        location: np.array,  # [x, y, z]
        project_to_road: bool = False,
        lane_type=LaneType.Any,
    ):
        [x, y, z] = location
        waypoint = self._map.get_waypoint(
            Location(float(x), float(y), float(z)),
            project_to_road=project_to_road,
            lane_type=lane_type,
        )
        return waypoint
