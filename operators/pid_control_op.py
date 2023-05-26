""" 
# PID Control operator

`pid` control operator computes the command that needs to be executed to follow the given waypoints. 
It reacts to the car current speed and position in a way that accelerates or brake according to previous inputs.

## Inputs

- waypoints coordinates to follow.

## Outputs

- throttle, steering (rad) and braking.

## Graph Description

```yaml
  - id: pid_control_op
    operator:
      python: ../../operators/pid_control_op.py
      outputs:
        - control
      inputs:
        position: oasis_agent/position
        speed: oasis_agent/speed
        waypoints: fot_op/waypoints
```

## Graph Viz

```mermaid
        flowchart TB
  oasis_agent
subgraph fot_op
  fot_op/op[op]
end
subgraph pid_control_op
  pid_control_op/op[op]
end
  oasis_agent -- position --> pid_control_op/op
  oasis_agent -- speed --> pid_control_op/op
  fot_op/op -- waypoints --> pid_control_op/op
  pid_control_op/op -- control --> oasis_agent
```

## Hyperparameters consider changing

See: https://en.wikipedia.org/wiki/PID_controller

```
pid_p = 0.1
pid_d = 0.0
pid_i = 0.05
dt = 1.0 / 20   
```
"""
import math
import time
from collections import deque
from typing import Callable

import numpy as np
import pyarrow as pa
from dora import DoraStatus
from numpy import linalg as LA
from scipy.spatial.transform import Rotation as R
from sklearn.metrics import pairwise_distances

pa.array([])  # See: https://github.com/apache/arrow/issues/34994

MIN_PID_WAYPOINT_DISTANCE = 1
pid_p = 0.4
pid_d = 0.0
pid_i = 0.05
dt = 1.0 / 20
pid_use_real_time = False

BRAKE_MAX = 1.0
THROTTLE_MAX = 0.5


def get_angle(left, right) -> float:
    """Computes the angle between the vector and another vector
    in radians."""

    angle = left - right
    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return angle


def compute_throttle_and_brake(pid, current_speed: float, target_speed: float):
    """Computes the throttle/brake required to reach the target speed.

    It uses the longitudinal controller to derive the required information.

    Args:
        pid: The pid controller.
        current_speed (:obj:`float`): The current speed of the ego vehicle
            (in m/s).
        target_speed (:obj:`float`): The target speed to reach (in m/s).

    Returns:
        Throttle and brake.
    """
    if current_speed < 0:
        print("Current speed is negative: {}".format(current_speed))
        non_negative_speed = 0
    else:
        non_negative_speed = current_speed
    acceleration = pid.run_step(target_speed, non_negative_speed)
    if acceleration >= 0.0:
        throttle = min(acceleration, THROTTLE_MAX)
        brake = 0
    else:
        throttle = 0.0
        brake = min(abs(acceleration), BRAKE_MAX)
    # Keep the brake pressed when stopped or when sliding back on a hill.
    if (current_speed < 1 and target_speed == 0) or current_speed < -0.3:
        brake = 1.0
    return throttle, brake


class PIDLongitudinalController(object):
    """Implements longitudinal control using a PID.

    Args:
       K_P (:obj:`float`): Proportional term.
       K_D (:obj:`float`): Differential term.
       K_I (:obj:`float`): Integral term.
       dt (:obj:`float`): time differential in seconds.
    """

    def __init__(
        self,
        K_P: float = 1.0,
        K_D: float = 0.0,
        K_I: float = 0.0,
        dt: float = 0.03,
        use_real_time: bool = False,
    ):
        self._k_p = K_P
        self._k_d = K_D
        self._k_i = K_I
        self._dt = dt
        self._use_real_time = use_real_time
        self._last_time = time.time()
        self._error_buffer = deque(maxlen=10)

    def run_step(self, target_speed: float, current_speed: float):
        """Computes the throttle/brake based on the PID equations.

        Args:
            target_speed (:obj:`float`): Target speed in m/s.
            current_speed (:obj:`float`): Current speed in m/s.

        Returns:
            Throttle, brake and steering.
        """
        # Transform to km/h
        error = (target_speed - current_speed) * 3.6
        self._error_buffer.append(error)

        if self._use_real_time:
            time_now = time.time()
            dt = time_now - self._last_time
            self._last_time = time_now
        else:
            dt = self._dt
        if len(self._error_buffer) >= 2:
            _de = (self._error_buffer[-1] - self._error_buffer[-2]) / dt
            _ie = sum(self._error_buffer) * dt
        else:
            _de = 0.0
            _ie = 0.0

        return np.clip(
            (self._k_p * error) + (self._k_d * _de) + (self._k_i * _ie),
            -1.0,
            1.0,
        )


pid = PIDLongitudinalController(pid_p, pid_d, pid_i, dt, pid_use_real_time)


class Operator:
    """
    Compute the throttle, target angle and brake given a `position`, a `speed` and a `waypoints`.
    """

    def __init__(self):
        self.waypoints = []
        self.target_speeds = []
        self.metadata = {}
        self.position = []
        self.speed = []
        self.previous_position = []
        self.current_speed = []
        self.previous_time = time.time()

    def on_event(
        self,
        dora_event: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        """Handle input.
        Args:
            dora_input["id"]  (str): Id of the input declared in the yaml configuration
            dora_input["value"] (arrow.array(UInt8)): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """

        if "position" == dora_input["id"]:
            self.position = dora_input["value"].to_numpy().view(np.float32)
            return DoraStatus.CONTINUE

        elif dora_input["id"] == "speed":
            self.speed = np.array(dora_input["value"]).view(np.float32)
            return DoraStatus.CONTINUE

        elif "waypoints" == dora_input["id"]:
            waypoints = dora_input["value"].to_numpy().view(np.float32)
            waypoints = waypoints.reshape((-1, 3))

            self.target_speeds = waypoints[:, 2]
            self.waypoints = waypoints[:, :2]
            self.metadata = dora_input["metadata"]

        if len(self.position) == 0 or len(self.speed) == 0:
            return DoraStatus.CONTINUE

        if len(self.waypoints) == 0:
            send_output(
                "control",
                pa.array(
                    np.array([0, 0, 1], np.float16).view(np.uint8).ravel()
                ),
                self.metadata,
            )
            return DoraStatus.CONTINUE

        [x, y, _, rx, ry, rz, rw] = self.position
        [_, _, yaw] = R.from_quat([rx, ry, rz, rw]).as_euler(
            "xyz", degrees=False
        )
        distances = pairwise_distances(self.waypoints, np.array([[x, y]])).T[0]

        index = distances > MIN_PID_WAYPOINT_DISTANCE
        self.waypoints = self.waypoints[index]
        self.target_speeds = self.target_speeds[index]
        distances = distances[index]

        if len(self.waypoints) == 0:
            target_angle = 0
            target_speed = 0
        else:
            argmin_distance = np.argmin(distances)

            ## Retrieve the closest point to the steer distance
            target_location = self.waypoints[argmin_distance]

            target_speed = self.target_speeds[argmin_distance]

            ## Compute the angle of steering
            target_vector = target_location - [x, y]

            target_angle = get_angle(
                math.atan2(target_vector[1], target_vector[0]), yaw
            )

        throttle, brake = compute_throttle_and_brake(
            pid, LA.norm(self.speed), target_speed
        )

        send_output(
            "control",
            pa.array(
                np.array([throttle, target_angle, brake], np.float16).view(
                    np.uint8
                )
            ),
            self.metadata,
        )
        return DoraStatus.CONTINUE
