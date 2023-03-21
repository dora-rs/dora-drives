#! /bin/bash
export DESTINATION=`cat $DEST`
dora-coordinator --run-dataflow $YAML

