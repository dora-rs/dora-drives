import os
import random
import threading
import time
from datetime import datetime

import pylot.utils
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

from dora_watermark import load

# You can generate an API token from the "API Tokens Tab" in the UI
token = os.getenv("INFLUX_TOKEN")
token = "iit96Hkq0sYco2sHIuFCM5cU4I5srivYQafgbZgoGmG92gReT9Kao3rNH8b3KFlgPskStVvOOaOU5-LZY94dfA=="
org = "shavtao@gmail.com"
bucket = "DORA Test Bucket"
mutex = threading.Lock()
goal_location = pylot.utils.Location(234, 59, 39)

summary_id = random.randint(0, 1000000)
host = "host1"

points = []
counter = 0
client = InfluxDBClient(
    url="https://eu-central-1-1.aws.cloud2.influxdata.com",
    token=token,
    org=org,
)
write_api = client.write_api(write_options=SYNCHRONOUS)


def write_to_influxdb(points):
    write_api.write(bucket, org, points)


def dora_run(inputs):
    if "control_status" not in inputs.keys():
        return {}
    global mutex
    global points
    global counter
    mutex.acquire()

    current_time = datetime.utcnow()
    if "position" in inputs.keys():
        pose = load(inputs, "position")
        location = pose.transform.location
        points.append(
            Point("dora.pylot.test")
            .tag("host", host)
            .tag("id", summary_id)
            .field("goal_distance", goal_location.distance(location))
            .time(current_time, WritePrecision.NS)
        )
        points.append(
            Point("dora.pylot.test")
            .tag("host", host)
            .tag("id", summary_id)
            .field("x_coordinate", location.x / 100)
            .time(current_time, WritePrecision.NS)
        )
        points.append(
            Point("dora.pylot.test")
            .tag("host", host)
            .tag("id", summary_id)
            .field("y_coordinate", location.y / 100)
            .time(current_time, WritePrecision.NS)
        )

    if "obstacles" in inputs.keys():
        obstacles = load(inputs, "obstacles")
        points.append(
            Point("dora.pylot.test")
            .tag("host", host)
            .tag("id", summary_id)
            .field("obstacles", len(obstacles))
            .time(current_time, WritePrecision.NS)
        )

    counter += 1

    if counter % 5000:
        write_to_influxdb(points)

    mutex.release()
    return {}
