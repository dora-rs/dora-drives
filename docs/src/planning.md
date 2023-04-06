# Planning

To make the car drive itself we first need to plan the way we want to go.

## GPS 

To do this, we're going to use gps to trace the route from our current location to our target location. 

Carla `GlobalRoutePlanner` enables us to get the route from two points given a map. We have encapsulated this function within `operators/carla_gps_op.py`.

The following operator will compute the route from the current `position` to the `objective_waypoints` given an `opendrive` map. 

```yaml
  - id: carla_gps_op
    operator:
      python: ../../carla/carla_gps_op.py
      outputs:
        - gps_waypoints
      inputs:
        opendrive: oasis_agent/opendrive
        objective_waypoints: oasis_agent/objective_waypoints
        position: oasis_agent/position
```

> The waypoints are defined as a an array of `x, y, speed` as `float32` waypoints, with global coordinate.

## Planner

The GPS waypoints does not take into account obstacles. To avoid collision, we can implement a motion planner that can avoid obstacles. 

We're going to reuse a model called `fot` (Frenet Optimal Trajectory) as a black box, that take as input a starting location and a goal waypoints, as well as a list of obstacles and outputs the best waypoints to follow.

```yaml
  - id: fot_op
    operator:
      python: operators/fot_op.py
      outputs:
        - waypoints
      inputs:
        position: oasis_agent/position
        obstacles: obstacle_location_op/obstacles
        gps_waypoints: carla_gps_op/gps_waypoints
```

> More info here: [https://github.com/erdos-project/frenet_optimal_trajectory_planner](https://github.com/erdos-project/frenet_optimal_trajectory_planner)

To test both functionallities:

```bash
dora up
dora start graphs/oasis/oasis_agent_planning.yaml --attach
```

<p align="center">
    <img src="./planning.png" width="800">
</p>