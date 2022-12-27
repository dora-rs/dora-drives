#!/bin/bash

if [ -z "$CARLA_HOME" ]; then
    echo "Please set \$CARLA_HOME before running this script"
    exit 1
fi

if [ -z "$1" ]; then
    PORT=2000
else
    PORT=$1
fi


export VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json

${CARLA_HOME}/CarlaUE4.sh  -windowed -ResX=800 -ResY=600 -carla-server -world-port=$PORT -benchmark -fps=20 -nosound -quality-level=Low