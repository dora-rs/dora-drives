# Carla simulator

Let's try to use a car simulator to not only do perception but also control.

## Carla Setup

In the rest of the tutorial, we will accept that you have a carla simulator running at `localhost:2000` the default carla configuration.

To start a simulator make sure you have carla installed. You can check the [installation page](./installation.md) if you not sure if it's installed.

You can also a docker version or any other method provided by [Carla](https://carla.org/)

In case you have used the installation script. You should be able to run a carla simulator using 

```bash
./scripts/run_simulator.sh
```
> You can define the VULKAN interface you want to use, using the env variable `VK_ICD_FILENAMES`. 
> 
> Ex for NVIDIA: `export VK_ICD_FILENAMES="/usr/share/vulkan/icd.d/nvidia_icd.json"`

If you're using the OASIS platflorm, follow the OASIS platform to start and run your dataflow.

## Switching from the 

We can then switch from the webcam to the simulator in our graph.

```yaml
{{#include ../../graphs/tutorials/carla_waypoints.yaml}}
```

> I have removed the traffic sign operator to reduce GPU memory consumption.