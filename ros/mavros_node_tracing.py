#!/home/nvidia/Documents/dora-rover/venv/bin/python3
# license removed for brevity
import rospy
from mavros_msgs.msg import PositionTarget, OverrideRCIn
from geometry_msgs.msg import TwistStamped
from dora import Node
import numpy as np
import time

import typing
from opentelemetry import trace
from opentelemetry.exporter.jaeger.thrift import JaegerExporter
from opentelemetry.sdk.resources import SERVICE_NAME, Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.trace.propagation.tracecontext import (
    TraceContextTextMapPropagator,
)


def parse_context(string_context: str):
    result = {}
    for elements in string_context.split(";"):
        splits = elements.split(":")
        if len(splits) == 2:
            result[splits[0]] = splits[1]
        elif len(splits) == 1:
            result[splits[0]] = ""
    return result


CarrierT = typing.TypeVar("CarrierT")
propagator = TraceContextTextMapPropagator()

trace.set_tracer_provider(
    TracerProvider(resource=Resource.create({SERVICE_NAME: "mavros"}))
)
tracer = trace.get_tracer(__name__)
jaeger_exporter = JaegerExporter(
    agent_host_name="172.17.0.1",
    agent_port=6831,
)
span_processor = BatchSpanProcessor(jaeger_exporter)
trace.get_tracer_provider().add_span_processor(span_processor)


node = Node()

TARGET_SPEED = 1600


def talker():
    pub = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size=1)
    rospy.init_node("talker", anonymous=True)
    for input_id, value, metadata in node:
        carrier = parse_context(metadata["open_telemetry_context"])
        context = propagator.extract(carrier=carrier)
        with tracer.start_as_current_span(
            name="control", context=context
        ) as child_span:

            [angle] = np.frombuffer(value)
            target = OverrideRCIn()
            if angle < np.pi / 2 and angle > -np.pi / 2:
                target_rotation = int((angle + np.pi / 2) / (np.pi) * 1000) + 1000
                target.channels[0] = target_rotation
                target.channels[1] = TARGET_SPEED
            elif angle < -np.pi / 2:
                target_rotation = 1000
                target.channels[0] = target_rotation
                target.channels[1] = TARGET_SPEED
            else:
                target.channels[0] = 2000
                target.channels[1] = TARGET_SPEED

            # target.channels[2] = 100
            # target = PositionTarget()
            # target.coordinate_frame = 9
            # target.header.stamp = rospy.get_rostime()
            # target.type_mask = int("110111111100",2)
            # target.position.x = 0.9
            # target.position.y = -0.9
            # target.velocity.x = 0.1
            # target.velocity.y = -0.1
            # target.yaw = yaw
            pub.publish(target)
    print("stopping")
    target = OverrideRCIn()
    target.channels[0] = 2000
    target.channels[1] = 1550

    pub.publish(target)
    print("stopped")

    # rate.sleep()


if __name__ == "__main__":
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
