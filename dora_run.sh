#! /bin/bash
export DESTINATION=`cat $DEST`
# export DORA_JAEGER_TRACING=172.17.0.1:6831

/home/dora/workspace/simulate/team_code/dora-drives/upgrade_dora.sh

dora up
dora start $YAML --attach --hot-reload

