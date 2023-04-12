# Installation

### Hardware requirements

- NVIDIA GPU with CUDA

## From Docker Hub
### Environments

|Software|Version|Installation Guide|
|--------|-------|------------------|
|[nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)|20.10.18|[Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)|

To start the docker container:
```bash
docker pull haixuantao/dora-drives
./scripts/launch.sh -s -g tutorials/carla_full.yaml
```

> This docker image has been built with my setup and it might not work on all machines. In case it doesn't work. Please check the following `From Source`.

## Building Docker From Source

### Environments

|Software|Version|Installation Guide|
|--------|-------|------------------|
|[nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)|20.10.18|[nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)|
|Linux|Ubuntu 20.04.5 LTS||

For linux, run:
```bash
git clone git@github.com:dora-rs/dora-drives.git
cd dora-drives
./scripts/launch.sh -b -s -g tutorials/carla_full.yaml
```

> This script has been built with my setup and you might need to install further dependencies that I have not listed, and additional configuration for cross-compiling on other OS.
> 
> If you're having build difficulties with CUDA. Check out :https://github.com/pytorch/extension-cpp/issues/71#issuecomment-1061880626 and make sure to have the exact same daemon.
You will need to have `/etc/docker/daemon.json` to be exactly:

>```json
>{
>    "runtimes": {
>        "nvidia": {
>            "path": "nvidia-container-runtime",
>            "runtimeArgs": []
>        }
>    },
>    "default-runtime": "nvidia"
>}
>```
>
>And restart:
>
>```bash
>sudo systemctl restart docker
>```

## Using `dora-drives` without Docker

|Software|Version Tested|Installation Guide|
|--------|-------|------------------|
|Linux|Ubuntu 20.04.5 LTS||
|Miniconda|22.11.1|Check the Dockerfile|
|Pytorch|1.11|Installation below|
|Carla|Carla Leaderboard|Installation below in `scripts/install.sh`. Version: [Leaderboard Version](https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/Leaderboard/CARLA_Leaderboard_20.tar.gz)|
|NVIDIA Driver|515.86.01||
|CUDA|11.7||
|dora-rs|0.2.0|Installation below|


### Environments

### Installation

```bash
git clone git@github.com:dora-rs/dora-drives.git

export DORA_DEP_HOME=<PATH TO A PARENT FOLDER> # Ex: $HOME/Documents
export DORA_DEP_HOME=$HOME/Documents
export CARLA_HOME=$DORA_DEP_HOME/dependencies/CARLA_0.9.13
export PYLOT_HOME=$DORA_DEP_HOME
export PYTHONPATH=$PYTHONPATH:$DORA_DEP_HOME/dependencies:$DORA_DEP_HOME/dependencies/CARLA_0.9.13/PythonAPI/carla:$DORA_DEP_HOME/dependencies/Carsmos/simulate_py37


## Add missing linux dependencies
sudo apt-get -y update 
sudo apt-get -y install apt-utils git curl clang wget
sudo apt-get install -y cmake unzip libpng-dev libgeos-dev python3-opencv
sudo apt-get -y --fix-missing update && sudo apt-get install --fix-missing -y libcudnn8 ssh libqt5core5a libeigen3-dev cmake qtbase5-dev libpng16-16 libtiff5 python3-tk libgeos-dev vim build-essential libopenblas-dev libssl-dev 

## Installing dependencies
conda create -n dora3.7 python=3.7 -y
conda activate dora3.7
conda install pytorch=1.11.0 torchvision=0.12.0 cudatoolkit=11.3 -c pytorch -y
pip install --upgrade pip
pip install -r install_requirements.txt
pip install -r requirements.txt


chmod +x ./scripts/*
./scripts/install.sh

## Installing Carla
cd $DORA_DEP_HOME/dependencies/
wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.13.tar.gz 
tar -xvf CARLA_0.9.13.tar.gz
rm CARLA_0.9.13.tar.gz

## Installing dora
sudo wget https://github.com/dora-rs/dora/releases/download/v0.2.0/dora-v0.2.0-x86_64-Linux.zip && sudo unzip dora-v0.2.0-x86_64-Linux.zip -d ~/.local/bin 
``` 


### Uninstalling package

```bash
conda remove --name dora3.7 --all
sudo rm -rf $DORA_DEP_HOME/dependencies
rm ~/.local/bin/dora*
    ```