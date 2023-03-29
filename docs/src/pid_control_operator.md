# PID Control operator

`pid` control operator computes the command that needs to be executed to follow the `hybrid_astar` waypoints.

## Inputs

- waypoints coordinates to follow.

## Outputs

- Command control.

## Graph Description

```yaml
  - id: pid_control_op
    operator:
      python: ../../operators/pid_control_op.py
      outputs:
        - control
      inputs:
        position: oasis_agent/position
        speed: oasis_agent/speed
        waypoints: fot_op/waypoints
```

## Graph Viz

```mermaid
        flowchart TB
  oasis_agent
subgraph fot_op
  fot_op/op[op]
end
subgraph pid_control_op
  pid_control_op/op[op]
end
  oasis_agent -- position --> pid_control_op/op
  oasis_agent -- speed --> pid_control_op/op
  fot_op/op -- waypoints --> pid_control_op/op
  pid_control_op/op -- control --> oasis_agent
```

## Hyperparameters consider changing

See: https://en.wikipedia.org/wiki/PID_controller

```
pid_p = 0.1
pid_d = 0.0
pid_i = 0.05
dt = 1.0 / 20   
```

<p align="center">
<img src="https://upload.wikimedia.org/wikipedia/commons/c/c0/Change_with_Ki.png" width="800">
</p>