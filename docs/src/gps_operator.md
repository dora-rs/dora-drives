# GPS operator



## Graph Description

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

## Graph Viz

```mermaid
        flowchart TB
  oasis_agent
subgraph fot_op
  fot_op/op[op]
end
subgraph carla_gps_op
  carla_gps_op/op[op]
end
  oasis_agent -- objective_waypoints --> carla_gps_op/op
  oasis_agent -- opendrive --> carla_gps_op/op
  oasis_agent -- position --> carla_gps_op/op
  carla_gps_op/op -- gps_waypoints --> fot_op/op
```