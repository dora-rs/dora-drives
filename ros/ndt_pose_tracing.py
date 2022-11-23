#!/home/nvidia/Documents/dora-rover/venv/bin/python3

import sensor_msgs.point_cloud2 as pc2
import rospy
from sensor_msgs.msg import PointCloud2, Imu
from geometry_msgs.msg import PoseStamped
import numpy as np
from dora import Node
import time
from scipy.spatial.transform import Rotation as R


import typing
from opentelemetry import trace
from opentelemetry.exporter.jaeger.thrift import JaegerExporter
from opentelemetry.sdk.resources import SERVICE_NAME, Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.trace.propagation.tracecontext import (
    TraceContextTextMapPropagator,
)


def serialize_context(context: dict) -> str:
    output = ""
    for key, value in context.items():
        output += f"{key}:{value};"
    return output


CarrierT = typing.TypeVar("CarrierT")
propagator = TraceContextTextMapPropagator()

trace.set_tracer_provider(
    TracerProvider(resource=Resource.create({SERVICE_NAME: "ndt_pose"}))
)
tracer = trace.get_tracer(__name__)
jaeger_exporter = JaegerExporter(
    agent_host_name="172.17.0.1",
    agent_port=6831,
)
span_processor = BatchSpanProcessor(jaeger_exporter)
trace.get_tracer_provider().add_span_processor(span_processor)


node = Node()
initial_orientation = None
orientation = None

GOAL_LOCATION = np.array([[0, 0, 0], [3, 0, 0], [3, -8, 0]])


def imu_callback(data):

    global initial_orientation
    global orientation

    if initial_orientation is None:
        initial_orientation = R.from_quat(
            [
                data.orientation.x,
                data.orientation.y,
                data.orientation.z,
                data.orientation.w,
            ]
        )
        print(
            f"INITIAL ORIENTATION: {initial_orientation.as_euler('xyz', degrees=True)}"
        )

        abs_goal_location = initial_orientation.apply(GOAL_LOCATION)
        abs_goal_location = abs_goal_location[:, :2]
        node.send_output("gps_waypoints", abs_goal_location.tobytes())

    orientation = [
        data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w,
    ]

    # node.send_output("imu", orientation.tobytes())


start = time.time()


def pose_callback(data):
    global start
    global initial_orientation
    global orientation
    position = np.array(
        [
            data.pose.position.x,
            data.pose.position.y,
            data.pose.position.z,
        ]
    )
    if initial_orientation is None:
        return None

    position = initial_orientation.apply(position)
    position = np.concatenate([position, orientation])

    if time.time() - start > 1:
        with tracer.start_as_current_span("start") as _span:
            output = {}
            propagator.inject(output)
            metadata = {"open_telemetry_context": serialize_context(output)}
            node.send_output("position", position.tobytes(), metadata)
            start = time.time()


rospy.init_node("listener", anonymous=True)
rospy.Subscriber("mavros/imu/data", Imu, imu_callback)
rospy.Subscriber("current_pose", PoseStamped, pose_callback)

rospy.spin()
