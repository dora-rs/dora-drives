import logging
import threading

import numpy as np
from pylot.control.pid import PIDLongitudinalController
from sklearn.metrics import pairwise_distances

from dora_watermark import load

mutex = threading.Lock()
old_waypoints = None
MIN_PID_STEER_WAYPOINT_DISTANCE = 5
MIN_PID_SPEED_WAYPOINT_DISTANCE = 5
STEER_GAIN = 0.7
COAST_FACTOR = 1.75
pid_p = 1.0
pid_d = 0.0
pid_i = 0.05
dt = 1.0 / 10
pid_use_real_time = True
pid = PIDLongitudinalController(pid_p, pid_d, pid_i, dt, pid_use_real_time)

logger = logging.Logger("")


class Flags(object):
    pass


FLAGS = Flags()
FLAGS.brake_max = 1.0
FLAGS.throttle_max = 0.5


def radians_to_steer(rad: float, steer_gain: float):
    """Converts radians to steer input.

    Returns:
        :obj:`float`: Between [-1.0, 1.0].
    """
    steer = steer_gain * rad
    if steer > 0:
        steer = min(steer, 1)
    else:
        steer = max(steer, -1)
    return steer


def compute_throttle_and_brake(
    pid, current_speed: float, target_speed: float, flags, logger
):
    """Computes the throttle/brake required to reach the target speed.

    It uses the longitudinal controller to derive the required information.

    Args:
        pid: The pid controller.
        current_speed (:obj:`float`): The current speed of the ego vehicle
            (in m/s).
        target_speed (:obj:`float`): The target speed to reach (in m/s).
        flags (absl.flags): The flags object.

    Returns:
        Throttle and brake values.
    """
    if current_speed < 0:
        logger.warning("Current speed is negative: {}".format(current_speed))
        non_negative_speed = 0
    else:
        non_negative_speed = current_speed
    acceleration = pid.run_step(target_speed, non_negative_speed)
    if acceleration >= 0.0:
        throttle = min(acceleration, flags.throttle_max)
        brake = 0
    else:
        throttle = 0.0
        brake = min(abs(acceleration), flags.brake_max)
    # Keep the brake pressed when stopped or when sliding back on a hill.
    if (current_speed < 1 and target_speed == 0) or current_speed < -0.3:
        brake = 1.0
    return throttle, brake


def dora_run(inputs):
    global old_waypoints

    keys = inputs.keys()
    if "position" not in keys:
        return {}

    position = np.frombuffer(inputs["position"])
    [x, y, z, pitch, yaw, roll, current_speed] = position

    # Vehicle speed in m/s.
    if "waypoints" in keys:
        waypoints = np.frombuffer(inputs["waypoints"])
        waypoints = waypoints.reshape((-1, 3))
        target_speeds = waypoints[2]
        waypoints = waypoints[:2].T
    elif old_waypoints is not None:
        (waypoints, target_speeds) = old_waypoints
    else:
        return {}

    mutex.acquire()
    distances = pairwise_distances(waypoints, [[x, y]]).flatten()

    old_waypoints = (waypoints, target_speeds)

    ## Retrieve the closest point to the steer distance
    expected_target_locations = waypoints[
        distances > MIN_PID_STEER_WAYPOINT_DISTANCE
    ]
    if len(expected_target_locations) == 0:
        target_angle = 0
    else:
        target_location = expected_target_locations[0]

        ## Compute the angle of steering
        forward_vector = target_location - [x, y]
        target_angle = np.arctan2(forward_vector[1], forward_vector[0])

    # Retrieve the target speed.
    expected_target_speed = target_speeds[
        distances > MIN_PID_SPEED_WAYPOINT_DISTANCE
    ]
    if len(expected_target_speed) == 0:
        target_speed = 0
    else:
        target_speed = expected_target_speed[0]

    throttle, brake = compute_throttle_and_brake(
        pid, current_speed, target_speed, FLAGS, logger
    )

    steer = radians_to_steer(target_angle, STEER_GAIN)

    mutex.release()

    return {
        "control": np.array([throttle, steer, brake]).tobytes(),
    }
