#! /bin/bash
export DESTINATION=`cat $DEST`
dora-daemon --run-dataflow $YAML

