#! /bin/bash
export DESTINATION=`cat $DEST`
dora up
dora start $YAML --attach # --hot-reload

