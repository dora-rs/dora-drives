#!/bin/bash
# InfluxDB
export INFLUX_TOKEN=RmC0MizRVFt04yBDXE12nqBIwUVvBEVb1qAYBbX3OFwZYSnpiGPPc03SUGfM4MSwprlknimeXOqWGb6CXWq8ZA==
~/workspace/dora-drives/telegraf-1.22.4/usr/bin/telegraf --config https://eu-central-1-1.aws.cloud2.influxdata.com/api/v2/telegrafs/09671055edbf6000 &

cd /home/dora/workspace/dora-drives
export PYTHONPATH=$PYTHONPATH:$(pwd)/nodes:$(pwd)/operators
export DORA_RUNTIME_PATH=/home/dora/workspace/dora-drives/bin/dora-runtime
export PATH=$PATH:/home/dora/workspace/dora-drives/bin/
export DORA_PATH=/home/dora/workspace/dora-drives
dora-coordinator run $DORA_PATH/graphs/drives_node_graph.yaml $DORA_RUNTIME_PATH
# ./dora-drives start-python sink_eval_plot plot image waypoints obstacles pose &
