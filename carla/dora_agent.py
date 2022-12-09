from enum import Enum

import carla

from leaderboard.autoagents.autonomous_agent import AutonomousAgent

import numpy as np
from scipy.spatial.transform import Rotation as R
from dora import Node
import cv2

# node = Node()
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600
STEER_GAIN = 0.7

node = Node()


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


class Track(Enum):

    """
    This enum represents the different tracks of the CARLA AD leaderboard.
    """

    SENSORS = "SENSORS"
    MAP = "MAP"


def get_entry_point():
    return "DoraAgent"


class DoraAgent(AutonomousAgent):
    def setup(self, path_to_conf_file):
        """
        Initialize everything needed by your agent and set the track attribute to the right type:
        Track.SENSORS : CAMERAS, LIDAR, RADAR, GPS and IMU sensors are allowed
        Track.MAP : OpenDRIVE map is also allowed
        """
        # self.track = Track.MAP

    def sensors(self):  # pylint: disable=no-self-use
        """
        Define the sensor suite required by the agent
        return: a list containing the required sensors in the following format:
        [
        {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        width': 300, 'height': 200, 'fov': 100, 'id': 'Left'},
        {'type': 'sensor.camera.rgb', 'x': 0.7, 'y': 0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
        width': 300, 'height': 200, 'fov': 100, 'id': 'Right'},
        {'type': 'sensor.lidar.ray_cast', 'x': 0.7, 'y': 0.0, 'z': 1.60, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0,
        id': 'LIDAR'}
        ]
        """
        sensors = [
            {
                "type": "sensor.camera.rgb",
                "id": "camera.center",
                "x": 0.0,
                "y": 0.0,
                "z": 1.50,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "width": IMAGE_WIDTH,
                "height": IMAGE_HEIGHT,
                "fov": 90,
            },
            # {'type': 'sensor.lidar.ray_cast', 'id': 'LIDAR',
            #'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0},
            # {'type': 'sensor.other.radar', 'id': 'RADAR',
            #'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0, 'yaw': -45.0, 'fov': 30},
            {
                "type": "sensor.other.gnss",
                "id": "GPS",
                "x": 0,
                "y": 0,
                "z": 1.5,
            },
            {
                "type": "sensor.other.imu",
                "id": "IMU",
                "x": 0,
                "y": 0,
                "z": 1.5,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            # {
            # "type": "sensor.opendrive_map",
            # "id": "OpenDRIVE",
            # "reading_frequency": 1,
            # },
            # {"type": "sensor.speedometer", "id": "Speed"},
        ]

        return sensors

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        :return: control
        """

        ## Implement send input data
        [x, y, z] = input_data["GPS"][1]
        yaw = input_data["IMU"][1][-1]
        roll = 0
        pitch = 0
        [[qx, qy, qz, qw]] = R.from_euler(
            "xyz", [[roll, pitch, yaw]], degrees=False
        ).as_quat()

        position = np.array([x, y, z, qx, qy, qz, qw], np.float32)

        frame_raw_data = input_data["camera.center"][1]

        frame = np.frombuffer(frame_raw_data, dtype=np.dtype("uint8"))
        frame = np.reshape(frame, (IMAGE_HEIGHT, IMAGE_WIDTH, 4))

        camera_frame = cv2.imencode(".jpg", frame)[1].tobytes()
        ## Implement receive

        input_id, value, metadata = node.next()

        control = np.frombuffer(value)

        steer = radians_to_steer(control[2], STEER_GAIN)
        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 0.0
        control.hand_brake = False

        return control
