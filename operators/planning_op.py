import logging
import threading
import time
from typing import Callable

import numpy as np
from carla import Client
from hybrid_astar_planner.HybridAStar.hybrid_astar_wrapper import (
    apply_hybrid_astar,
)

from dora_utils import DoraStatus
from dora_world import World
from hd_map import HDMap

mutex = threading.Lock()


class Flags(object):
    pass


FLAGS = Flags()
FLAGS.tracking_num_steps = 10
FLAGS.planning_type = "rrt"
FLAGS.max_speed = 10.0
FLAGS.max_accel = 6.0
FLAGS.max_curvature = 1.0

# Hybrid AStar flags
FLAGS.step_size_hybrid_astar = 3.0
FLAGS.max_iterations_hybrid_astar = 2000
FLAGS.completion_threshold = 1.0
FLAGS.angle_completion_threshold = 100
FLAGS.rad_step = 4.0
FLAGS.rad_upper_range = 4.0
FLAGS.rad_lower_range = 4.0
FLAGS.obstacle_clearance_hybrid_astar = 1.0
FLAGS.lane_width_hybrid_astar = 6.0
FLAGS.radius = 6.0
FLAGS.car_length = 4.0
FLAGS.car_width = 1.8

FLAGS.dynamic_obstacle_distance_threshold = 50

# Planning general
FLAGS.target_speed = 10.0
FLAGS.obstacle_radius = 1.0
FLAGS.num_waypoints_ahead = 30
FLAGS.num_waypoints_behind = 0
FLAGS.obstacle_filtering_distance = 1.0

# RRT Star
FLAGS.step_size = 0.5
FLAGS.max_iterations = 200
FLAGS.end_dist_threshold = 2.0
FLAGS.obstacle_clearance_rrt = 0.5
FLAGS.lane_width = 3.0

# FOT
FLAGS.max_speed = 35.0  # Maximum vehicle speed [m/s]
FLAGS.max_accel = 6.0  # Maximum vehicle acceleration [m/s^2]
FLAGS.max_curvature = 1.0  # Maximum curvature speed [1/m]
FLAGS.max_road_width_l = 5.0  # Maximum left road width [m]
FLAGS.max_road_width_r = 1.0  # Maximum right road width [m]
FLAGS.d_road_w = 0.25  # Road width sampling discretization [m]
FLAGS.dt = 0.25  # Time sampling discretization [s]
FLAGS.maxt = 8.0  # Max prediction horizon [s]
FLAGS.mint = 2.0  # Min prediction horizon [s]
FLAGS.d_t_s = 0.25  # Target speed sampling discretization [m/s]
FLAGS.n_s_sample = 2.0  # Number speeds to sample
FLAGS.obstacle_clearance_fot = 0.5  # 'Obstacle clearance threshold [m]'
FLAGS.kd = 1.0  # Deviation cost
FLAGS.kv = 0.1  # Velocity cost
FLAGS.ka = 0.1  # Acceleration cost
FLAGS.kj = 0.01  # Jerk cost
FLAGS.kt = 0.01  # Time cost
FLAGS.ko = 0.1  # Obstacle cost
FLAGS.klat = 1.0  # Lateral cost
FLAGS.klon = 1.0  # Longitudinal cost

goal_location = [234, 59, 39]
CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"

logger = logging.getLogger("")
time.sleep(5)  # Wait for the world to load.
world = World(FLAGS, logger)
world._goal_location = goal_location
client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
client.set_timeout(30.0)  # seconds
carla_world = client.get_world()
hd_map = HDMap(carla_world.get_map())


class HybridAStarPlanner:
    """Wrapper around the Hybrid A* planner.

    Note:
        Details can be found at `Hybrid A* Planner`_.

    Args:
        world: (:py:class:`~pylot.planning.world.World`): A reference to the
            planning world.
        flags (absl.flags): Object to be used to access absl flags.

    .. _Hybrid A* Planner:
       https://github.com/erdos-project/hybrid_astar_planner
    """

    def __init__(self, world, flags, logger):
        self._flags = flags
        self._logger = logger
        self._world = world
        # TODO: Deal with the map
        self._map = None
        self._hyperparameters = {
            "step_size": flags.step_size_hybrid_astar,
            "max_iterations": flags.max_iterations_hybrid_astar,
            "completion_threshold": flags.completion_threshold,
            "angle_completion_threshold": flags.angle_completion_threshold,
            "rad_step": flags.rad_step,
            "rad_upper_range": flags.rad_upper_range,
            "rad_lower_range": flags.rad_lower_range,
            "obstacle_clearance": flags.obstacle_clearance_hybrid_astar,
            "lane_width": flags.lane_width_hybrid_astar,
            "radius": flags.radius,
            "car_length": flags.car_length,
            "car_width": flags.car_width,
        }

    def run(self, timestamp, ttd=None):
        """Runs the planner.

        Note:
            The planner assumes that the world is up-to-date.

        Returns:
            :py:class:`~pylot.planning.waypoints.Waypoints`: Waypoints of the
            planned trajectory.
        """
        obstacle_list = self._world.get_obstacle_list()

        if len(obstacle_list) == 0:
            # Do not use Hybrid A* if there are no obstacles.
            return self._world.follow_waypoints(self._flags.target_speed)

        # Hybrid a* does not take into account the driveable region.
        # It constructs search space as a top down, minimum bounding
        # rectangle with padding in each dimension.

        initial_conditions = self._compute_initial_conditions(obstacle_list)
        path_x, path_y, _, success = apply_hybrid_astar(
            initial_conditions, self._hyperparameters
        )

        if not success:
            return self._world.follow_waypoints(0)

        speeds = np.array([self._flags.target_speed] * len(path_x))

        output_wps = np.array([path_x, path_y]).T

        return output_wps, speeds

    def _compute_initial_conditions(self, obstacles):
        [x, y, _, _, yaw, _, _] = self._world.position
        start = np.array(
            [
                x,
                y,
                np.deg2rad(yaw),
            ]
        )
        end_index = min(
            self._flags.num_waypoints_ahead,
            len(self._world.waypoints) - 1,
        )

        if end_index < 0:
            # If no more waypoints left. Then our location is our end wp.
            end = np.array(
                [
                    x,
                    y,
                    np.deg2rad(yaw),
                ]
            )

        else:
            [end_x, end_y] = self._world.waypoints[end_index]
            end = np.array(
                [
                    end_x,
                    end_y,
                    np.deg2rad(yaw),
                ]
            )

        initial_conditions = {
            "start": start,
            "end": end,
            "obs": obstacles,
        }
        return initial_conditions


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.obstacles = []
        self.position = []
        self.planner = HybridAStarPlanner(world, FLAGS, logger)

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ):

        if input_id == "position":
            self.position = np.frombuffer(value)

        if "obstacles" == input_id:
            obstacles = value.split(b"\n")
            obstacles_prediction = []
            for obstacle in obstacles:
                if len(obstacle) % 12 == 0:
                    obstacle_buffer = np.frombuffer(obstacle, dtype="float32")

                    obstacle_position = np.reshape(obstacle_buffer, (-1, 3))
                    obstacles_prediction.append(obstacle_position)
            self.obstacles = obstacles_prediction
            return DoraStatus.CONTINUE

        self.planner._world.update(
            time.time(), self.position, self.obstacles, [], hd_map
        )

        (waypoints, target_speeds) = self.planner.run(
            time.time()
        )  # , open_drive)
        self.planner._world.waypoints = waypoints

        waypoints_array = np.concatenate(
            [waypoints.T, target_speeds.reshape(1, -1)]
        )

        send_output(
            "waypoints",
            waypoints_array.tobytes(),
        )
        return DoraStatus.CONTINUE
