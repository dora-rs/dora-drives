#!/opt/conda/envs/dora3.8/bin/python

import logging
import time
import zlib

import cv2
import numpy as np
from dora import Node
import typing
from opentelemetry import trace
from opentelemetry.exporter.jaeger.thrift import JaegerExporter
from opentelemetry.sdk.resources import SERVICE_NAME, Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.trace.propagation.tracecontext import (
    TraceContextTextMapPropagator,
)

from _generate_world import (
    add_camera,
    add_depth_camera,
    add_lidar,
    add_segmented_camera,
    spawn_actors,
    spawn_driving_vehicle,
)
from carla import Client, Location, Rotation, Transform


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
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600

lidar_pc = None
depth_frame = None
camera_frame = None
segmented_frame = None


sensor_transform = Transform(
    Location(3, 0, 1), Rotation(pitch=0, yaw=0, roll=0)
)


def on_segmented_msg(frame):

    frame = np.frombuffer(frame.raw_data, dtype=np.dtype("uint8"))
    frame = np.reshape(frame, (IMAGE_HEIGHT, IMAGE_WIDTH, 4))
    frame = frame[:, :, 2]
    global segmented_frame
    segmented_frame = cv2.imencode(".jpg", frame)[1].tobytes()


def on_lidar_msg(frame):

    global lidar_pc
    lidar_pc = zlib.compress(frame.raw_data.tobytes())


def on_camera_msg(frame):
    frame = np.frombuffer(frame.raw_data, dtype=np.dtype("uint8"))
    frame = np.reshape(frame, (IMAGE_HEIGHT, IMAGE_WIDTH, 4))

    global camera_frame
    camera_frame = cv2.imencode(".jpg", frame)[1].tobytes()


def on_depth_msg(frame):

    frame = np.frombuffer(
        frame.raw_data,
        dtype="uint8",
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


def main():

    global camera_frame
    global segmented_frame
    global depth_frame
    global lidar_pc

    if camera_frame is None or segmented_frame is None or depth_frame is None:
        return {}

    vec_transform = ego_vehicle.get_transform()
    velocity_vector = ego_vehicle.get_velocity()
    x = vec_transform.location.x
    y = vec_transform.location.y
    z = vec_transform.location.z
    pitch = vec_transform.rotation.pitch
    yaw = vec_transform.rotation.yaw
    roll = vec_transform.rotation.roll

    vx = velocity_vector.x
    vy = velocity_vector.y
    vz = velocity_vector.z

    forward_speed = np.linalg.norm([vx, vy, vz])

    position = np.array([x, y, z, pitch, yaw, roll, forward_speed])
    with tracer.start_as_current_span("source") as _span:
        output = {}
        propagator.inject(output)
        metadata = {"open_telemetry_context": serialize_context(output)}
        node.send_output("position", position.tobytes(), metadata)
        node.send_output("image", camera_frame, metadata)
        node.send_output("depth_frame", depth_frame, metadata)
        node.send_output("segmented_frame", segmented_frame, metadata)
        node.send_output("lidar_pc", lidar_pc, metadata)


for _ in range(1000):
    time.sleep(1)
    main()
