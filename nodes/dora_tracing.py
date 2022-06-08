import abc
import re
import typing

from opentelemetry import trace
from opentelemetry.exporter.jaeger.thrift import JaegerExporter
from opentelemetry.sdk.resources import SERVICE_NAME, Resource
from opentelemetry.sdk.trace import TracerProvider
from opentelemetry.sdk.trace.export import BatchSpanProcessor
from opentelemetry.trace.propagation.tracecontext import (
    TraceContextTextMapPropagator,
)

CarrierT = typing.TypeVar("CarrierT")
propagator = TraceContextTextMapPropagator()

trace.set_tracer_provider(
    TracerProvider(resource=Resource.create({SERVICE_NAME: "python-client"}))
)
tracer = trace.get_tracer(__name__)
jaeger_exporter = JaegerExporter(
    agent_host_name="172.17.0.1",
    agent_port=6831,
)
span_processor = BatchSpanProcessor(jaeger_exporter)
trace.get_tracer_provider().add_span_processor(span_processor)


class Getter(abc.ABC):
    """This class implements a Getter that enables extracting propagated
    fields from a carrier.
    """

    def get(
        self, carrier: CarrierT, key: str
    ) -> typing.Optional[typing.List[str]]:
        """Function that can retrieve zero
        or more values from the carrier. In the case that
        the value does not exist, returns None.
        Args:
            carrier: An object which contains values that are used to
                    construct a Context.
            key: key of a field in carrier.
        Returns: first value of the propagation key or None if the key doesn't
                exist.
        """
        if key in carrier.keys():
            return [carrier[key]]

    def keys(self, carrier: CarrierT) -> typing.List[str]:
        """Function that can retrieve all the keys in a carrier object.
        Args:
            carrier: An object which contains values that are
                used to construct a Context.
        Returns:
            list of keys from the carrier.
        """
        return list(carrier.keys())


getter = Getter()


def parse_context(string_context: str) -> CarrierT:

    result = {}

    for elements in string_context.split(";"):
        splits = elements.split(":")
        if len(splits) == 2:
            result[splits[0]] = splits[1]
        elif len(splits) == 1:
            result[splits[0]] = ""

    return result


def extract_context(inputs):
    string_context = inputs["otel_context"].decode("utf-8")

    carrier = parse_context(string_context)

    context = propagator.extract(carrier=carrier, getter=getter)

    return context
