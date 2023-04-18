import math
import os.path
import xml.etree.ElementTree as ET

import numpy as np
import pyarrow as pa
from autoagents.autonomous_agent import AutonomousAgent
from dora import Node
from dora_tracing import propagator, serialize_context, tracer
from scipy.spatial.transform import Rotation as R

pa.array([])  # See: https://github.com/apache/arrow/issues/34994
from carla import VehicleControl

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
STEER_GAIN = 1
AVERAGE_WINDOW = 10

node = Node()


def _get_latlon_ref(xodr):
    """
    Convert from waypoints world coordinates to CARLA GPS coordinates
    :return: tuple with lat and lon coordinates
    """
    tree = ET.ElementTree(ET.fromstring(xodr))

    # default reference
    lat_ref = 42.0
    lon_ref = 2.0

    for opendrive in tree.iter("OpenDRIVE"):
        for header in opendrive.iter("header"):
            for georef in header.iter("geoReference"):
                if georef.text:
                    str_list = georef.text.split(" ")
                    for item in str_list:
                        if "+lat_0" in item:
                            lat_ref = float(item.split("=")[1])
                        if "+lon_0" in item:
                            lon_ref = float(item.split("=")[1])
    return lat_ref, lon_ref


def from_gps_to_world_coordinate(lat, lon, lat_ref, lon_ref):

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
    x = mx - mx_initial
    y = -(my - my_initial)

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


def get_entry_point():
    return "DoraAgent"


class DoraAgent(AutonomousAgent):
    def setup(self, destination, path_to_conf_file):
        """
        Setup the agent parameters
        """
        self.previous_positions = []
        self.destination = destination
        self.lat_ref = None
        self.lon_ref = None
        self.opendrive_map = None
        self.last_metadata = ""

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
                "x": 2,
                "y": 0,
                "z": 1.5,
            },
            {
                "type": "sensor.other.imu",
                "id": "IMU",
                "x": 2,
                "y": 0,
                "z": 1.5,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            {
                "type": "sensor.opendrive_map",
                "id": "高精地图传感器",
                "reading_frequency": 1,
            },
            {"type": "sensor.speedometer", "id": "速度传感器"},
        ]

        return sensors

    def save_input_data(self, keys, inputdata):
        import json

        data = {keys: inputdata}
        opendrive_file = "/home/dora/workspace/simulate/inputdata_log.txt"
        if os.path.exists(opendrive_file):
            os.remove(opendrive_file)
        with open(opendrive_file, "w") as f:
            f.write(json.dumps(data))
            f.close()

    def run_step(self, input_data, timestamp):
        """
        Execute one step of navigation.
        :return: control
        """
        with tracer.start_as_current_span(name="run_step") as child_span:
            output = {}
            propagator.inject(output)
            metadata = {"open_telemetry_context": serialize_context(output)}
            ### Opendrive preprocessing
            if "高精地图传感器" in input_data.keys():

                if self.opendrive_map is None:
                    opendrive_map = input_data["高精地图传感器"][1]["opendrive"]
                    self.save_input_data("高精地图传感器", input_data["高精地图传感器"])

                    self.opendrive_map = opendrive_map
                    self.lat_ref, self.lon_ref = _get_latlon_ref(opendrive_map)
                    node.send_output(
                        "opendrive", opendrive_map.encode(), metadata
                    )
            if "速度传感器" in input_data.keys():
                node.send_output(
                    "speed",
                    pa.array(
                        np.array(
                            input_data["速度传感器"][1]["speed"], np.float32
                        ).view(np.uint8)
                    ),
                    metadata,
                )

            if self.lat_ref is None:
                return VehicleControl(
                    steer=0.0,
                    throttle=0.0,
                    brake=0.0,
                    hand_brake=False,
                )

            ### Position preprocessing
            [lat, lon, z] = input_data["GPS"][1]
            [x, y] = from_gps_to_world_coordinate(
                lat, lon, self.lat_ref, self.lon_ref
            )
            yaw = input_data["IMU"][1][-1] - np.pi / 2
            roll = 0.0
            pitch = 0.0
            [[qx, qy, qz, qw]] = R.from_euler(
                "xyz", [[roll, pitch, yaw]], degrees=False
            ).as_quat()
            try:
                R.from_quat([qx, qy, qz, qw])
            except:
                print("Error in quaternion.")
                return VehicleControl(
                    steer=0.0,
                    throttle=0.0,
                    brake=0.0,
                    hand_brake=False,
                )

            self.previous_positions.append([x, y])

            ## Accumulate previous position until having window size average.
            if len(self.previous_positions) < AVERAGE_WINDOW:
                return VehicleControl(
                    steer=0.0,
                    throttle=0.0,
                    brake=0.0,
                    hand_brake=False,
                )
            self.previous_positions = self.previous_positions[-AVERAGE_WINDOW:]

            ## Average last 5 position
            [avg_x, avg_y] = np.array(self.previous_positions).mean(axis=0)
            position = np.array([avg_x, avg_y, 0.0, qx, qy, qz, qw], np.float32)

            ### Camera preprocessing
            frame_raw_data = input_data["camera.center"][1]
            ## frame = np.frombuffer(frame_raw_data, np.uint8)
            ## frame = np.reshape(frame, (IMAGE_HEIGHT, IMAGE_WIDTH, 4))
            camera_frame = pa.array(frame_raw_data.view(np.uint8).ravel())

            ### LIDAR preprocessing
            frame_raw_data = input_data["LIDAR"][1]
            frame = np.frombuffer(frame_raw_data, np.float32)
            point_cloud = np.reshape(frame, (-1, 4))
            point_cloud = point_cloud[:, :3]
            lidar_pc = pa.array(point_cloud.view(np.uint8).ravel())

            ### Waypoints preprocessing
            waypoints_xyz = np.array(
                [
                    [
                        self.destination.x,
                        self.destination.y,
                        0.0,
                    ]
                ],
                np.float32,
            )

            ## Sending data into the dataflow
            node.send_output(
                "position",
                pa.array(position.view(np.uint8).ravel()),
                metadata,
            )
            node.send_output("image", camera_frame, metadata)
            node.send_output("lidar_pc", lidar_pc, metadata)
            node.send_output(
                "objective_waypoints",
                pa.array(waypoints_xyz.view(np.uint8).ravel()),
                metadata,
            )

            # Receiving back control information
            ## Using tick to avoid deadlock due to unreceived input.
            for iteration in range(5):
                event = node.next()
                if event["type"] == "INPUT":
                    input_id = event["id"]
                    value = event["value"]

                    if input_id == "tick" and iteration > 0 and iteration < 4:
                        print(
                            f"Did not receive control after {iteration} ticks..."
                        )
                    elif input_id == "tick" and iteration == 4:
                        print(
                            f"Sending null control after waiting {iteration} ticks..."
                        )
                        value = np.array([0.0, 0.0, 0.0], np.float16)
                    elif input_id == "control":
                        break

            [throttle, target_angle, brake] = np.array(value).view(np.float16)

            steer = radians_to_steer(target_angle, STEER_GAIN)
            vec_control = VehicleControl(
                steer=float(steer),
                throttle=float(throttle),
                brake=float(brake),
                hand_brake=False,
            )

            return vec_control
