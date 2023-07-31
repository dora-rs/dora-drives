<p align="center">
    <img src="./docs/src/logo.svg" width="400">
</p>

---

`dora-drives` is a step-by-step tutorial that allows beginners to write their own autonomous vehicle program from scratch using a simple starter kit.

## Why dora-drives?

We believe that programming autonomous driving vehicles is the perfect starting point to learning robotic applications as:
- Autonomous driving is foundational for many robotic applications.
- Autonomous driving is simple to explain.
- There are a lot of datasets, models and documentation available online.

## Installation

```bash
git clone git@github.com:dora-rs/dora-drives.git
cd dora-drives

## Installing dependencies
conda create -n dora3.7 python=3.7 -y
conda activate dora3.7
conda install pytorch=1.11.0 torchvision=0.12.0 cudatoolkit=11.3 -c pytorch -y
pip install --upgrade pip
pip install -r install_requirements.txt
pip install -r requirements.txt

## Installing dora if its not already installed
sudo wget https://github.com/dora-rs/dora/releases/download/v0.2.4/dora-v0.2.4-x86_64-Linux.zip && sudo unzip dora-v0.2.4-x86_64-Linux.zip -d /usr/local/bin 
```

For more info, see: https://dora.carsmos.ai/docs/guides/dora-drives/installation

## Getting started

You can run a fully looped autonomous vehicle with just the following command:

```bash
docker pull carlasim/carla:0.9.13

# Serve the carla simulator
docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh -carla-server -world-port=2000 -RenderOffScreen

# Spawn dora daemon and coordinator
dora up

# Spawn dora dataflow
dora start graphs/oasis/oasis_full.yaml --attach
```

To get a step-by-step tutorial, see: https://dora.carsmos.ai/docs/guides/dora-drives/carla

## Documentation

The documentation can be found here: https://dora.carsmos.ai/docs/guides/dora-drives

## Discussion

Our main communication channel is our Github Project Discussion page: https://github.com/orgs/dora-rs/discussions

Feel free to reach out on any topic, issues or ideas.

## ⚖️ LICENSE 

This project is licensed under Apache-2.0. Check out [NOTICE.md](NOTICE.md) for more information.
