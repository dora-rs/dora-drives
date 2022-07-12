import threading

import numpy as np
from carla import Client, VehicleControl, command

from dora_tracing import extract_context, tracer

CARLA_SIMULATOR_HOST = "localhost"
CARLA_SIMULATOR_PORT = "2000"
client = Client(CARLA_SIMULATOR_HOST, int(CARLA_SIMULATOR_PORT))
client.set_timeout(30.0)
vehicle_id = None

mutex = threading.Lock()


def dora_run(inputs):
    global vehicle_id

    if vehicle_id is None and "vehicle_id" not in inputs.keys():
        return {}
    elif vehicle_id is None and "vehicle_id" in inputs.keys():
        global mutex
        mutex.acquire()
        vehicle_id = int.from_bytes(inputs["vehicle_id"], "big")
        mutex.release()

    if "control" not in inputs.keys():
        return {}

    # Control should be a C-order array with Throttle, steer, and brake as value.
    control = np.frombuffer(inputs["control"])

    vec_control = VehicleControl(
        throttle=control[0],
        steer=control[1],
        brake=control[2],
        hand_brake=False,
        reverse=False,
    )

    context = extract_context(inputs)

    with tracer.start_span(
        f"within-python-{__name__}", context=context
    ) as span:
        client.apply_batch_sync(
            [command.ApplyVehicleControl(vehicle_id, vec_control)]
        )
        return {"control_status": (1).to_bytes(2, "big")}
