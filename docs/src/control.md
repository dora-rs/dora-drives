# Control

Car can simplistically be controlled in general using 3 variables: `throttle, steering, brake`. We're going to focus on those 3 for the moment.

To send a control to a Carla car, we can use `carla_control_op.py` that takes an array of 3 variables: `throttle, steering, brake` and apply it to our car.

To translate out waypoints to those control, we're using a PID controller that is able to adjust the steering according to the response of the steering, and we're going to pipe this response to the CARLA API so that the car can move. The PID controller code is in `pid_control_op.py`.

The full graph look as follows:

```yaml
{{#include ../../graphs/tutorials/carla_full.yaml}}
```

You can visualize your graph with:
```bash
dora graph graphs/tutorials/carla_full.yaml --open                 
 ```

```mermaid
        flowchart TB
  carla_source_node
subgraph carla_gps_op
  carla_gps_op/op[op]
end
subgraph yolov5
  yolov5/op[op]
end
subgraph obstacle_location_op
  obstacle_location_op/op[op]
end
subgraph fot_op
  fot_op/op[op]
end
subgraph pid_control_op
  pid_control_op/op[op]
end
subgraph plot
  plot/op[/op\]
end
subgraph carla_control_op
  carla_control_op/op[op]
end
subgraph ___dora___ [dora]
  subgraph ___timer_timer___ [timer]
    dora/timer/millis/400[\millis/400/]
    dora/timer/millis/500[\millis/500/]
  end
end
  dora/timer/millis/500 -- tick --> carla_source_node
  carla_source_node -- objective_waypoints --> carla_gps_op/op
  carla_source_node -- position --> carla_gps_op/op
  carla_source_node -- image --> yolov5/op
  missing>missing] -- lanes --> obstacle_location_op/op
  carla_source_node -- lidar_pc --> obstacle_location_op/op
  yolov5/op -- bbox as obstacles_bbox --> obstacle_location_op/op
  carla_source_node -- position --> obstacle_location_op/op
  obstacle_location_op/op -- global_lanes --> fot_op/op
  carla_gps_op/op -- gps_waypoints --> fot_op/op
  obstacle_location_op/op -- obstacles --> fot_op/op
  carla_source_node -- position --> fot_op/op
  carla_source_node -- position --> pid_control_op/op
  fot_op/op -- waypoints --> pid_control_op/op
  pid_control_op/op -- control --> plot/op
  missing>missing] -- drivable_area --> plot/op
  obstacle_location_op/op -- global_lanes --> plot/op
  carla_gps_op/op -- gps_waypoints --> plot/op
  carla_source_node -- image --> plot/op
  missing>missing] -- lanes --> plot/op
  obstacle_location_op/op -- obstacles --> plot/op
  yolov5/op -- bbox as obstacles_bbox --> plot/op
  missing>missing] -- obstacles_id --> plot/op
  carla_source_node -- position --> plot/op
  dora/timer/millis/400 -- tick --> plot/op
  missing>missing] -- traffic_sign_bbox --> plot/op
  fot_op/op -- waypoints --> plot/op
  pid_control_op/op -- control --> carla_control_op/op
  carla_source_node -- vehicle_id --> carla_control_op/op
```

To test it out:

```bash
./scripts/launch.sh -b -s -g tutorials/carla_full.yaml
```

- To run it without docker:

```bash
dora-daemon --run-dataflow graphs/tutorials/carla_full.yaml
```

😎 We now have a working autonomous car!