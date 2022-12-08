# Planning

To make the car drive itself we first need to plan the way we want to go.

## GPS 

To do this, we're going to use gps waypoints from our current location to our target location.

Luckily we have a `carla` gps operator. 

To get the gps waypoint with a preset target location. All we have to do is add:

```yaml
  - id: carla_gps_op
    operator:
      python: carla/carla_gps_op.py
      outputs:
        - gps_waypoints
      inputs:
        position: carla_source_node/position
```

It will compute waypoints from the input location to go to the preset target location. 

The waypoints are defined as a an array of `x, y, speed` as `float32` waypoints, with global coordinate.

## Planner

The GPS waypoints does not take into account obstacles. To avoid collision, we can implement a motion planner that can avoid obstacles. 

We're going to reuse a model called `hybrid_astar` as a black box, that take as input a starting location and a goal location, as well as a list of obstacles and he will be able to solve the best waypoints on its own.

```yaml
  - id: hybrid_astar_op
    operator:
      python: operators/hybrid_astar_op.py
      outputs:
        - waypoints
      inputs:
        position: carla_source_node/position
        obstacles: obstacle_location_op/obstacles
        gps_waypoints: carla_gps_op/gps_waypoints
```

To test waypoints algorithms, launch:

```bash
./scripts/launch.sh -b -s -g tutorials/carla_waypoints.yaml
```

- To run it without docker:

```bash
dora-coordinator --run-dataflow graphs/tutorials/carla_waypoints.yaml
```
