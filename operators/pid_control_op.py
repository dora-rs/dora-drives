import logging
import threading
from pickle import dumps, loads

import numpy as np
import pylot.control.utils
import pylot.planning.utils
from pylot.control.pid import PIDLongitudinalController

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

from typing import Callable


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.waypoints = None

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ):
        """Handle input.
        Args:
            input_id (str): Id of the input declared in the yaml configuration
            value (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """

        if "position" == input_id:
            pose = loads(value)
            ego_transform = pose.transform
            # Vehicle speed in m/s.
            current_speed = pose.forward_speed
        if "waypoints" == input_id:
            mutex.acquire()
            self.waypoints = loads(value)
            mutex.release()
            return {}

        if self.waypoints == None:
            return None
        mutex.acquire()

        try:
            angle_steer = self.waypoints.get_angle(
                ego_transform, MIN_PID_STEER_WAYPOINT_DISTANCE
            )

            target_speed = self.waypoints.get_target_speed(
                ego_transform, MIN_PID_SPEED_WAYPOINT_DISTANCE
            )
            mutex.release()

            throttle, brake = pylot.control.utils.compute_throttle_and_brake(
                pid, current_speed, target_speed, FLAGS, logger
            )

            steer = pylot.control.utils.radians_to_steer(
                angle_steer, STEER_GAIN
            )
        except ValueError:
            print("Braking! No more waypoints to follow.")
            throttle, brake = 0.0, 0.5
            steer = 0.0

        send_output("control", np.array([throttle, steer, brake]).tobytes())
