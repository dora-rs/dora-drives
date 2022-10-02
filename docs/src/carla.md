# Carla simulator

Let's try to use a car simulator to not only do perception but also control.

To launch the simulator all you have to do is add the `-s`flag in the `./script/launch.sh` command.

We can then switch from the webcam to the simulator in our graph.

```yaml
{{#include ../../graphs/tutorials/carla_perception.yaml}}
```

We can then tun it with the following command:

```bash
./scripts/launch.sh -b -s -g tutorials/carla_perception.yaml
```

> I have removed the traffic sign operator to reduce GPU memory consumption.