from typing import Callable
import typing
import cv2
import numpy as np
from enum import Enum

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
    TracerProvider(resource=Resource.create({SERVICE_NAME: "webcam"}))
)
tracer = trace.get_tracer(__name__)
jaeger_exporter = JaegerExporter(
    agent_host_name="172.17.0.1",
    agent_port=6831,
)
span_processor = BatchSpanProcessor(jaeger_exporter)
trace.get_tracer_provider().add_span_processor(span_processor)


class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def __init__(self):
        self.video_capture = cv2.VideoCapture(0)
        self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ):
        with tracer.start_as_current_span("start") as _span:
            output = {}
            propagator.inject(output)
            metadata = {"open_telemetry_context": serialize_context(output)}
            ret, frame = self.video_capture.read()
            frame = cv2.resize(frame, (640, 480))
            encode = cv2.imencode(".jpg", frame)[1].tobytes()
            send_output("image", encode, metadata)
            return DoraStatus.CONTINUE

    def drop_operator(self):
        self.video_capture.release()
