# Obstacle location

The carla simulator gives us the possibility to work with many more sensors than just a camera feed. We can emulate an LIDAR, IMU, Depth sensor, segmentation sensor...

Let's use the LIDAR sensor to locate the exact position of the obstacle that has been located by `yolov5`.

The lidar point cloud is an array of `x, y, z, intensity` points.

> The coordinates are based on Unreal Engine coordinate system which is: 
> - z is up
> - x is forward
> - y is right
> 
>  More info: [https://www.techarthub.com/a-practical-guide-to-unreal-engine-4s-coordinate-system/](https://www.techarthub.com/a-practical-guide-to-unreal-engine-4s-coordinate-system/)
> 
> and within carla documentation: [https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor](https://carla.readthedocs.io/en/latest/ref_sensors/#lidar-sensor)

To get the obstacle location, we are going to compute the angle of every points in the point cloud. We can then map the angle of each pixel of the bounding box to a real point and therefore infere its location. The code can be found here: [`operators/obstacle_location_op.py`](https://github.com/dora-rs/dora-drives/blob/main/operators/obstacle_location_op.py)

To use the obstacle location, just add it to the graph with:

```yaml
  - id: obstacle_location_op
    operator: 
      outputs:
        - obstacles
      inputs:
        lidar_pc: carla_source_node/lidar_pc
        obstacles_bbox: yolov5/bbox
        position: carla_source_node/position
      python: operators/obstacle_location_op.py
```
