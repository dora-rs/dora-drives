# Control

Car can simplistically be controlled in general using 3 variables: `throttle, steering, brake`. We're going to focus on those 3 for the moment.

To send a control to a Carla car, we can use `carla_control_op.py` that takes an array of 3 variables: `throttle, steering, brake` and apply it to our car.

To translate out waypoints to those control, we're using a PID controller that is able to adjust the steering according to the response of the steering, and we're going to pipe this response to the CARLA API so that the car can move. The PID controller code is in `pid_control_op.py`.

The full graph look as follows

```yaml
{{#include ../../graphs/tutorials/carla_full.yaml}}
```

```mermaid
flowchart TB
  carla_source_node[\carla_source_node/]
subgraph yolov5
  yolov5/op[op]
end
subgraph obstacle_location_op
  obstacle_location_op/op[op]
end
subgraph carla_gps_op
  carla_gps_op/op[op]
end
subgraph hybrid_astar_op
  hybrid_astar_op/op[op]
end
subgraph pid_control_op
  pid_control_op/op[op]
end
subgraph carla_control_op
  carla_control_op/op[op]
end
subgraph plot_op
  plot_op/op[/op\]
end
  carla_source_node -- image --> yolov5/op
  carla_source_node -- lidar_pc --> obstacle_location_op/op
  yolov5/op -- bbox as obstacles_bbox --> obstacle_location_op/op
  carla_source_node -- position --> obstacle_location_op/op
  carla_source_node -- position --> carla_gps_op/op
  carla_gps_op/op -- gps_waypoints --> hybrid_astar_op/op
  obstacle_location_op/op -- obstacles --> hybrid_astar_op/op
  carla_source_node -- position --> hybrid_astar_op/op
  carla_source_node -- position --> pid_control_op/op
  hybrid_astar_op/op -- waypoints --> pid_control_op/op
  pid_control_op/op -- control --> carla_control_op/op
  carla_source_node -- vehicle_id --> carla_control_op/op
  carla_source_node -- image --> plot_op/op
  obstacle_location_op/op -- obstacles --> plot_op/op
  yolov5/op -- bbox as obstacles_bbox --> plot_op/op
  carla_source_node -- position --> plot_op/op
  hybrid_astar_op/op -- waypoints --> plot_op/op
```

To test it out:

```bash
./scripts/launch.sh -b -s -g tutorials/carla_full.yaml
```

- To run it without docker:

```bash
dora-coordinator --run-dataflow graphs/tutorials/carla_full.yaml
```

😎 We now have a working autonomous car!