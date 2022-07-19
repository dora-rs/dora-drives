import logging
import threading
import time

import numpy as np

from dora_hybrid_astar_planner import HybridAStarPlanner
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

logger = logging.getLogger("")


def get_world(host: str = "localhost", port: int = 2000, timeout: int = 10):
    """Get a handle to the world running inside the simulation.

    Args:
        host (:obj:`str`): The host where the simulator is running.
        port (:obj:`int`): The port to connect to at the given host.
        timeout (:obj:`int`): The timeout of the connection (in seconds).

    Returns:
        A tuple of `(client, world)` where the `client` is a connection to the
        simulator and `world` is a handle to the world running inside the
        simulation at the host:port.
    """
    try:
        from carla import Client

        client = Client(host, port)
        client_version = client.get_client_version()
        server_version = client.get_server_version()
        err_msg = "Simulator client {} does not match server {}".format(
            client_version, server_version
        )
        assert client_version == server_version, err_msg
        client.set_timeout(timeout)
        world = client.get_world()
    except RuntimeError as r:
        raise Exception(
            "Received an error while connecting to the "
            "simulator: {}".format(r)
        )
    except ImportError:
        raise Exception("Error importing CARLA.")
    return (client, world)


def get_map(host: str = "localhost", port: int = 2000, timeout: int = 10):
    """Get a handle to the CARLA map.

    Args:
        host (:obj:`str`): The host where the simulator is running.
        port (:obj:`int`): The port to connect to at the given host.
        timeout (:obj:`int`): The timeout of the connection (in seconds).

    Returns:
        carla.Map: A map of the CARLA town.
    """
    _, world = get_world(host, port, timeout)
    return world.get_map()


time.sleep(5)  # Wait for the world to load.
world = World(FLAGS, logger)
world._goal_location = goal_location
hd_map = HDMap(get_map())
planner = HybridAStarPlanner(world, FLAGS, logger)
old_obstacles = None


def dora_run(inputs):
    global old_obstacles

    keys = inputs.keys()
    if "position" not in keys:  # or "open_drive" not in keys:
        return {}

    global mutex
    mutex.acquire()
    position = np.frombuffer(inputs["position"])
    [x, y, z, pitch, yaw, roll, current_speed] = position

    if "obstacles" in keys:
        obstacles = inputs["obstacles"]
        obstacles = obstacles.split(b"\n")
    elif old_obstacles is not None:
        obstacles = old_obstacles
    else:
        obstacles = []

    # open_drive = inputs,"open_drive"].decode("utf-8")
    global planning
    old_obstacles = obstacles
    planner._world.update(time.time(), position, [], [], hd_map)
    (waypoints, target_speeds) = planner.run(time.time())  # , open_drive)
    waypoints_array = np.concatenate(
        [waypoints.T, target_speeds.reshape(1, -1)]
    )
    mutex.release()

    return {
        "waypoints": waypoints_array.tobytes(),
    }
