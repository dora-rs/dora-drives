#!/bin/bash
# InfluxDB
export INFLUX_TOKEN=RmC0MizRVFt04yBDXE12nqBIwUVvBEVb1qAYBbX3OFwZYSnpiGPPc03SUGfM4MSwprlknimeXOqWGb6CXWq8ZA==
~/workspace/dora-rs/telegraf-1.22.4/usr/bin/telegraf --config https://eu-central-1-1.aws.cloud2.influxdata.com/api/v2/telegrafs/09671055edbf6000 &

cd /home/dora/workspace/dora-rs
export PYTHONPATH=$PYTHONPATH:$(pwd)/nodes
export DORA_RUNTIME_PATH=/home/dora/workspace/dora-rs/bin/dora-runtime
./bin/dora-coordinator run ./graphs/pylot_graph.yaml
# ./dora-rs start-python sink_eval_plot plot image waypoints obstacles pose &
