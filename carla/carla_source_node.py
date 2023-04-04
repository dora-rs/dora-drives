#!/usr/bin/env python

import logging
import typing
import zlib

import cv2
import numpy as np
from _generate_world import (add_camera, add_depth_camera, add_lidar,
                             add_segmented_camera, spawn_actors,
                             spawn_driving_vehicle)
from dora import Node
from numpy import linalg as LA
from opentelemetry import trace
from opentelemetry.exporter.jaeger.thrift import JaegerExporter
from opentelemetry.sdk.resources import SERVICE_NAME, Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.trace.propagation.tracecontext import \
    TraceContextTextMapPropagator
from scipy.spatial.transform import Rotation as R

from carla import Client, Location, Rotation, Transform


def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.cos(pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(
        roll / 2
    ) * np.sin(pitch / 2) * np.sin(yaw / 2)
    return [qx, qy, qz, qw]


def serialize_context(context: dict) -> str:
    output = ""
    for key, value in context.items():
        output += f"{key}:{value};"
    return output


logger = logging.Logger("")
CarrierT = typing.TypeVar("CarrierT")
propagator = TraceContextTextMapPropagator()

trace.set_tracer_provider(
    TracerProvider(
        resource=Resource.create({SERVICE_NAME: "carla_source_node"})
    )
)
tracer = trace.get_tracer(__name__)
jaeger_exporter = JaegerExporter(
    agent_host_name="172.17.0.1",
    agent_port=6831,
)
span_processor = BatchSpanProcessor(jaeger_exporter)
trace.get_tracer_provider().add_span_processor(span_processor)


CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"
LABELS = "image"
IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
OBJECTIVE_WAYPOINTS = np.array([[234, 59, 39]], np.float32)

lidar_pc = None
depth_frame = None
camera_frame = None
segmented_frame = None
last_position = np.array([0., 0.])

sensor_transform = Transform(
    Location(3, 0, 1), Rotation(pitch=0, yaw=0, roll=0)
)


def on_segmented_msg(frame):

    frame = np.frombuffer(frame.raw_data, np.uint8)
    frame = np.reshape(frame, (IMAGE_HEIGHT, IMAGE_WIDTH, 4))
    frame = frame[:, :, 2]
    global segmented_frame
    segmented_frame = cv2.imencode(".jpg", frame)[1].tobytes()


def on_lidar_msg(frame):

    global lidar_pc
    frame = np.frombuffer(frame.raw_data, np.float32)
    point_cloud = np.reshape(frame, (-1, 4))
    point_cloud = point_cloud[:, :3]

    lidar_pc = point_cloud.tobytes()


def on_camera_msg(frame):
    # frame = np.frombuffer(frame.raw_data, np.uint8)
    # frame = np.reshape(frame, (IMAGE_HEIGHT, IMAGE_WIDTH, 4))

    global camera_frame
    camera_frame = np.frombuffer(frame.raw_data, np.uint8).tobytes()


def on_depth_msg(frame):

    frame = np.frombuffer(
        frame.raw_data,
        np.uint8,
    )
    frame = np.reshape(frame, (IMAGE_HEIGHT, IMAGE_WIDTH, 4))
    frame = frame.astype(np.float32)
    frame = np.dot(frame[:, :, :3], [65536.0, 256.0, 1.0])
    frame /= 16777215.0  # (256.0 * 256.0 * 256.0 - 1.0)
    frame = frame.astype(np.float32)
    global depth_frame
    depth_frame = zlib.compress(frame.tobytes())


client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
client.set_timeout(30.0)  # seconds
world = client.get_world()

(_, _, _) = spawn_actors(
    client,
    world,
    8000,
    "0.9.10",
    -1,
    True,
    0,
    0,
    logger,
)


ego_vehicle, vehicle_id = spawn_driving_vehicle(client, world)
lidar = add_lidar(world, sensor_transform, on_lidar_msg, ego_vehicle)
depth_camera = add_depth_camera(
    world, sensor_transform, on_depth_msg, ego_vehicle
)
camera = add_camera(world, sensor_transform, on_camera_msg, ego_vehicle)
segmented_camera = add_segmented_camera(
    world, sensor_transform, on_segmented_msg, ego_vehicle
)

node = Node()

node.send_output("vehicle_id", vehicle_id.to_bytes(2, "big"))
node.send_output("opendrive", world.get_map().to_opendrive().encode())


def main():
    global last_position
    if camera_frame is None or segmented_frame is None or depth_frame is None:
        return {}

    vec_transform = ego_vehicle.get_transform()
    x = vec_transform.location.x
    y = vec_transform.location.y
    z = vec_transform.location.z
    yaw = vec_transform.rotation.yaw
    pitch = vec_transform.rotation.pitch
    roll = vec_transform.rotation.roll

    [[qx, qy, qz, qw]] = R.from_euler(
        "xyz", [[roll, pitch, yaw]], degrees=True
    ).as_quat()

    position = np.array([x, y, z, qx, qy, qz, qw], np.float32)
    # with tracer.start_as_current_span("source") as _span:
    output = {}
    propagator.inject(output)
    metadata = {"open_telemetry_context": serialize_context(output)}
    node.send_output("position", position.tobytes(), metadata)
    node.send_output(
        "speed",
        np.array([LA.norm(position[:2] - last_position[:2])]).tobytes(),
        metadata,
    )
    node.send_output("image", camera_frame, metadata)
    node.send_output(
        "objective_waypoints", OBJECTIVE_WAYPOINTS.tobytes(), metadata
    )
    # node.send_output("depth_frame", depth_frame, metadata)
    # node.send_output("segmented_frame", segmented_frame, metadata)
    node.send_output("lidar_pc", lidar_pc, metadata)
    last_position = position


for event in node:
    main()
