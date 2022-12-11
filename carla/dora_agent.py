from enum import Enum

import carla

from leaderboard.autoagents.autonomous_agent import AutonomousAgent
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

import numpy as np
from scipy.spatial.transform import Rotation as R
from dora import Node
import cv2
import math
import xml.etree.ElementTree as ET

# node = Node()
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600
STEER_GAIN = 0.7

node = Node()


def _get_latlon_ref(world):
    """
    Convert from waypoints world coordinates to CARLA GPS coordinates
    :return: tuple with lat and lon coordinates
    """

    # default reference
    lat_ref = 42.0
    lon_ref = 2.0

    return lat_ref, lon_ref


lat_ref, lon_ref = _get_latlon_ref(CarlaDataProvider.get_world())


def from_gps_to_world_coordinate(lat, lon):
    global lat_ref, lon_ref

    EARTH_RADIUS_EQUA = 6378137.0  # pylint: disable=invalid-name
    scale = math.cos(lat_ref * math.pi / 180.0)
    mx_initial = scale * lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
    my_initial = (
        scale
        * EARTH_RADIUS_EQUA
        * math.log(math.tan((90.0 + lat_ref) * math.pi / 360.0))
    )

    # lon = mx * 180.0 / (math.pi * EARTH_RADIUS_EQUA * scale)
    # lat =   360.0 * math.atan(math.exp(my / (EARTH_RADIUS_EQUA * scale))) / math.pi - 90.0

    mx = lon / 180.0 * (math.pi * EARTH_RADIUS_EQUA * scale)
    my = math.log(math.tan((lat + 90.0) * math.pi / 360.0)) * (
        EARTH_RADIUS_EQUA * scale
    )
    x = -(mx - mx_initial)
    y = my - my_initial

    return [x, y]


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
    # def setup(self, path_to_conf_file):
    # """
    # Initialize everything needed by your agent and set the track attribute to the right type:
    # Track.SENSORS : CAMERAS, LIDAR, RADAR, GPS and IMU sensors are allowed
    # Track.MAP : OpenDRIVE map is also allowed
    # """
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
                "x": 2.0,
                "y": 0.0,
                "z": 1.5,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "width": IMAGE_WIDTH,
                "height": IMAGE_HEIGHT,
                "fov": 90,
            },
            {
                "type": "sensor.lidar.ray_cast",
                "id": "LIDAR",
                "x": 2.0,
                "y": 0.0,
                "z": 1.5,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
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

        ### Position preprocessing
        [lat, lon, z] = input_data["GPS"][1]
        [x, y] = from_gps_to_world_coordinate(lat, lon)
        yaw = input_data["IMU"][1][-1]
        roll = 0
        pitch = 0
        [[qx, qy, qz, qw]] = R.from_euler(
            "xyz", [[roll, pitch, yaw]], degrees=False
        ).as_quat()
        position = np.array([x, y, z, qx, qy, qz, qw], np.float32)

        ### Camera preprocessing
        frame_raw_data = input_data["camera.center"][1]
        frame = np.frombuffer(frame_raw_data, dtype=np.dtype("uint8"))
        frame = np.reshape(frame, (IMAGE_HEIGHT, IMAGE_WIDTH, 4))
        camera_frame = cv2.imencode(".jpg", frame)[1].tobytes()

        ### LIDAR preprocessing
        frame_raw_data = input_data["LIDAR"][1]
        frame = np.frombuffer(frame_raw_data, dtype=np.dtype("float32"))
        point_cloud = np.reshape(frame, (-1, 4))
        point_cloud = point_cloud[:, :3]
        lidar_pc = point_cloud.tobytes()

        ### Waypoints preprocessing
        waypoints_xy = np.array(
            [
                [transform[0].location.x, transform[0].location.y]
                for transform in self._global_plan_world_coord
            ]
        )
        waypoints_xyv = np.hstack(
            (waypoints_xy, 0.5 + np.zeros((waypoints_xy.shape[0], 1)))
        ).astype(np.float32)

        ## Sending data into the dataflow
        node.send_output("position", position.tobytes())
        node.send_output("image", camera_frame)
        node.send_output("lidar_pc", lidar_pc)
        node.send_output("gps_waypoints", waypoints_xyv.tobytes())

        ## Receiving back control information
        input_id, value, metadata = node.next()

        control = np.frombuffer(value)

        steer = radians_to_steer(control[2], STEER_GAIN)
        vec_control = carla.VehicleControl()
        vec_control.steer = steer
        vec_control.throttle = control[0]
        vec_control.brake = control[1]
        vec_control.hand_brake = False

        return vec_control
