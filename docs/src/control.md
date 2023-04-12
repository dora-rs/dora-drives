# Control

## PID Controller 

To translate our waypoints to `throttle, steering and brake` control, we're using a Proportional Integral Derivative (PID) controller that is able to adjust the throttle, steering and breaking according to the car position and speed by comparing it to the desired waypoints. The code can be found in `operator/pid_control_op.py`.

> For more information on `pid`, go on [our `pid` detail page](./pid_control_operator.md)

## Control

The actual command being applied to the car is controlled within the `oasis_agent`.

## Fully looped graph

We have now all our starter kit node. They will look like this:

```yaml
# graphs/oasis/oasis_agent.yaml

{{#include ../../graphs/oasis/oasis_agent.yaml}}
```

To run a running car example:
```bash
dora up
dora start graphs/oasis/oasis_agent.yaml --attach
```

😎 We now have a working autonomous car!

You might have noticed that improvement can be done in many place.

In case you need inspiration, we advise you check:
- `operators/yolop_op.py` that enables you to detect lanes. It can be passed to the obstacle location to get the 3D position of the lanes. Those 3D position of lanes can then be passed to `fot` to plan by taking into account lanes on the floor.
- `operators/strong_sort.py` that enables tracking 2D bounding box through times. This can be useul if you want to avoid moving vehicles.
- `opertators/traffic_sign.py` that is self-trained traffic light detection based on yolov7 and tt100k. THis can be useful to avoid traffic light.