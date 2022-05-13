#!/bin/bash
cd /home/erdos/workspace/dora-rs
export PYTHONPATH=$PYTHONPATH:$(pwd)/nodes
# export RUST_LOG=warn
./dora-rs start-python carla_source_node send &
./dora-rs start-python perfect_detection_node run pose depth_frame segmented_frame &
./dora-rs start-python obstacle_location_node run pose depth_frame obstacles_without_location &
./dora-rs start-python planning_node run pose obstacles &
./dora-rs start-python pid_control_node run pose waypoints &
./dora-rs start-python control_node run control vehicle_id &
# ./dora-rs start-python summary_node run control_status pose obstacles &
# ./dora-rs start-python sink_eval_plot plot image waypoints obstacles pose &
# for i in 1 2 3 4 5; do ./dora-rs start-python carla_source_node send && break || sleep 10; done