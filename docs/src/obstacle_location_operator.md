# Obstacle location operator

The obstacle location operator match bounding box with depth frame to find the exact position of obstacles.

## Inputs

- Obstacles bounding box.

## Outputs

- GPS location of obstacles.


## Graph Description

```yaml
  - id: obstacle_location_op
    operator: 
      outputs:
        - obstacles
      inputs:
        lidar_pc: oasis_agent/lidar_pc
        obstacles_bbox: yolov5/bbox
        position: oasis_agent/position
      python: ../../operators/obstacle_location_op.py
```

## Graph Viz

```mermaid
        flowchart TB
  oasis_agent
subgraph yolov5
  yolov5/op[op]
end
subgraph fot_op
  fot_op/op[op]
end
subgraph obstacle_location_op
  obstacle_location_op/op[op]
end
  oasis_agent -- lidar_pc --> obstacle_location_op/op
  yolov5/op -- bbox as obstacles_bbox --> obstacle_location_op/op
  oasis_agent -- position --> obstacle_location_op/op
  obstacle_location_op/op -- obstacles --> fot_op/op
```