import numpy as np
from hybrid_astar_planner.HybridAStar.hybrid_astar_wrapper import (
    apply_hybrid_astar,
)


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
