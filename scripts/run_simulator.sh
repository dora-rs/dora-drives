#!/bin/bash

if [ -z "$CARLA_HOME" ]; then
    echo "Please set \$CARLA_HOME before running this script"
    exit 1
fi

${CARLA_HOME}/CarlaUE4.sh ResX=800 -ResY=600 -carla-server -world-port=2000 -benchmark -fps=5 -nosound -quality-level=Low  -RenderOffScreen
