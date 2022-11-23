#!/usr/bin/env python3
import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
from dora import Node
import numpy as np

node = Node()


async def run():
    drone = System()
    print("Waiting for drone...")
    await drone.connect(system_address="serial:///dev/ttyACM0:57600")
    print("Waiting for drone...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state}")
            break

    for input_id, value, metadata in node:

        [vx, vy, vz, yaw] = np.frombuffer(value)
        print(f"vel: {vx, vy, vz, yaw}")
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.1, 0.1, 0, 45))
        print("awaited!")
        # await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0, 0, 0.0, 0.0))
        # await asyncio.sleep(0.5)

    print("-- Landing")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
