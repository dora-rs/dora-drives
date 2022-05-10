#!/bin/bash
cd /home/erdos/workspace/dora-rs
export PYTHONPATH=$PYTHONPATH:$(pwd)/nodes
./dora-rs start-python carla_source_operator send &
./dora-rs start-python perfect_detection_operator run pose depth_frame segmented_frame &
./dora-rs start-python obstacle_location_operator run pose depth_frame obstacles_without_location &
./dora-rs start-python planning_operator run pose obstacles &
./dora-rs start-python pid_control_operator run pose waypoints &
./dora-rs start-python control_operator run control vehicle_id &
# ./dora-rs start-python summary_operator run control_status pose obstacles &
# ./dora-rs start-python sink_eval_plot plot image waypoints obstacles pose &