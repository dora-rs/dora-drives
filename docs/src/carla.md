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

- To run it without docker:

```bash
## Make sure your environment variable are well set.
export DORA_DEP_HOME=<PATH TO A PARENT FOLDER> # Ex: $HOME/Documents
export CARLA_HOME=$DORA_DEP_HOME/dependencies/CARLA_0.9.10.1
export PYLOT_HOME=$DORA_DEP_HOME
export PYTHONPATH=$PYTHONPATH:$DORA_DEP_HOME/dependencies:$DORA_DEP_HOME/dependencies/CARLA_0.9.10.1/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:$DORA_DEP_HOME/dependencies/CARLA_0.9.10.1/PythonAPI/carla/:$DORA_DEP_HOME/dependencies/CARLA_0.9.10.1/PythonAPI/carla/agents/:$DORA_DEP_HOME/dependencies/CARLA_0.9.10.1/PythonAPI/

## First start the simulator if it is not started
./scripts/run_simulator.sh &

dora-coordinator --run-dataflow graphs/tutorials/carla_perception.yaml
```

> I have removed the traffic sign operator to reduce GPU memory consumption.