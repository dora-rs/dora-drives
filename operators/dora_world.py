import numpy as np
from sklearn.metrics import pairwise_distances

from dora_utils import closest_vertex, distance_vertex

# Number of predicted locations to consider when computing speed factors.
NUM_FUTURE_TRANSFORMS = 10
TARGET_SPEED = 10.0
OBSTACLE_RADIUS = 1.0
NUM_WAYPOINTS_AHEAD = 30
NUM_WAYPOINTS_BEHIND = 0
OBSTACLE_FILTERING_DISTANCE = 1.0
STATIC_OBSTACLE_DISTANCE_THRESHOLD = 70
OBSTACLE_DISTANCE_WAYPOINTS_THRESHOLD = 10


class World(object):
    """A representation of the world that is used by the planners."""

    def __init__(self):
        self.static_obstacles = None
        self.obstacle_predictions = []
        self._ego_obstacle_predictions = []
        self.position = []
        self.ego_trajectory = []
        self.ego_transform = None
        self.ego_velocity_vector = None
        self._lanes = None
        self._map = None
        self._goal_location = []
        self.waypoints = np.array([])
        self.target_speeds = []
        self.timestamp = None
        self._last_stop_ego_location = None
        self._distance_since_last_full_stop = 0
        self._num_ticks_stopped = 0

    def update(
        self,
        timestamp,
        position,
        obstacle_predictions,
        static_obstacles,
        hd_map=None,
        lanes=None,
    ):
        self.timestamp = timestamp
        self.position = position
        self.ego_trajectory.append(self.position)
        self.obstacle_predictions = obstacle_predictions
        self._ego_obstacle_predictions = obstacle_predictions
        # Tranform predictions to world frame of reference.
        # Road signs are in world coordinates. Only maintain the road signs
        # that are within the threshold.
        self.static_obstacles = []
        for obstacle in static_obstacles:
            if (
                obstacle.transform.location.distance(
                    self.ego_transform.location
                )
                <= STATIC_OBSTACLE_DISTANCE_THRESHOLD
            ):
                self.static_obstacles.append(obstacle)

        if hd_map is None and lanes is None:
            print(
                "@{}: planning world does not have lanes or HD map".format(
                    timestamp
                )
            )
        self._map = hd_map
        self._lanes = lanes

        # The waypoints are not received on the global trajectory stream.
        # We need to compute them using the map.
        if len(self.waypoints) < NUM_WAYPOINTS_AHEAD / 2:
            if self._map is not None and self._goal_location is not None:
                waypoints = hd_map.compute_waypoints(
                    position[:3], self._goal_location
                )[:NUM_WAYPOINTS_AHEAD]
                self.target_speeds = np.array(
                    [0 for _ in range(len(self.waypoints))]
                )
                self.waypoints = waypoints
        else:

            (index, _) = closest_vertex(
                self.waypoints,
                np.array([self.position[:2]]),
            )

            self.waypoints = self.waypoints[index : index + NUM_WAYPOINTS_AHEAD]
            self.target_speeds = self.target_speeds[
                index : index + NUM_WAYPOINTS_AHEAD
            ]

        if position[-1] < 0.7:  # Speed check
            # We can't just check if forward_speed is zero because localization
            # noise can cause the forward_speed to be non zero even when the
            # ego is stopped.
            self._num_ticks_stopped += 1
            if self._num_ticks_stopped > 10:
                self._distance_since_last_full_stop = 0
                self._last_stop_ego_location = self.position
        else:
            self._num_ticks_stopped = 0
            if self._last_stop_ego_location is not None:
                self._distance_since_last_full_stop = distance_vertex(
                    self._last_stop_ego_location, self.position
                )
            else:
                self._distance_since_last_full_stop = 0

    def follow_waypoints(self, target_speed: float):

        if target_speed is not None:
            self.target_speeds = np.array(
                [target_speed for _ in range(len(self.waypoints))]
            )

        return self.waypoints, self.target_speeds

    def get_obstacle_list(self):
        obstacle_list = []
        if len(obstacle_list) == 0:
            return np.empty((0, 4))

        obstacles = np.array(self.obstacle_predictions).reshape((-1, 3))
        distances = pairwise_distances(self.waypoints, obstacles[-1, :2])
        for distance, prediction in zip(distances, self.obstacle_predictions):
            # Use all prediction times as potential obstacles.
            if distance < OBSTACLE_DISTANCE_WAYPOINTS_THRESHOLD:
                for location in prediction:
                    [x, y, _] = location
                    obstacle_size = np.array(
                        [
                            x - OBSTACLE_RADIUS,
                            y - OBSTACLE_RADIUS,
                            x + OBSTACLE_RADIUS,
                            y + OBSTACLE_RADIUS,
                        ]
                    )
                    obstacle_list.append(obstacle_size)
        return np.array(obstacle_list)
