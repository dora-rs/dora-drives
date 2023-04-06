# Obstacle location operator

The obstacle location operator match bounding box with depth frame to find an approximative position of obstacles.

There is two logic within it:
- One is for the ground dot for lane detection.
- One is for bounding box obstacle localisation.

Both logic are based on he computation of the projection in 2D space of the lidar 3D point and then reusing the index to get the 3D position.

- In the case of ground dot detection, the approximation is based on a knnr, as we might not have enough data on the floor.
- In the case of bounding box, we use first quantile closest point within the bounding box to estimate the distance. We use the first quantile closest point to remove the noise.

The mecanism to project the lidar point cloud into a 2D is also used in the `plot.py` operator. You can use the input `lidar_pc` within it to help you debug.

## Inputs

- 2D Obstacles bounding box.

## Outputs

- 3D position of obstacles as dot.


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