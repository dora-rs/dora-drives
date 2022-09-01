FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04
MAINTAINER Xavier Tao (tao.xavier@outlook.com)

# Set up a dora user first.

## See: https://github.com/NVIDIA/nvidia-docker/issues/1632
RUN apt-key del 7fa2af80
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub
RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu2004/x86_64/7fa2af80.pub
RUN apt-get -y update && apt-get -y install sudo
ENV uid 1000
ENV gid 1000

RUN mkdir -p /home/dora
RUN groupadd dora -g ${gid} 
RUN useradd -r -u ${uid} -g dora dora
RUN echo "dora ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/dora
RUN chmod 0440 /etc/sudoers.d/dora
RUN chown ${uid}:${gid} -R /home/dora
RUN usermod --shell /bin/bash dora


USER dora
ENV HOME /home/dora
ENV SHELL /bin/bash
WORKDIR /home/dora
SHELL ["/bin/bash", "--login", "-c"]

RUN mkdir -p /home/dora/workspace
RUN cd /home/dora/workspace

RUN sudo apt-get -y update && sudo apt-get -y install --reinstall locales && sudo locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US
ENV LC_ALL en_US.UTF-8

# Install tzdata without prompt.
RUN sudo apt-get -y --fix-missing update
ENV DEBIAN_FRONTEND=noninteractive
RUN sudo DEBIAN_FRONTEND=noninteractive sudo DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata

# Get the dora package dependencies.
RUN sudo apt-get -y install apt-utils git curl clang python-is-python3 python3-pip
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install setuptools setuptools-rust numpy==1.19.5

# Setup Rust and install dora.
RUN mkdir -p /home/dora/workspace

RUN sudo apt-get -y --fix-missing update && sudo apt-get install --fix-missing -y libcudnn8 ssh libqt5core5a libeigen3-dev cmake qtbase5-dev libpng16-16 libtiff5 python3-tk python3-pygame libgeos-dev vim

RUN mkdir -p /home/dora/workspace/dora_dependencies

WORKDIR /home/dora/workspace/dora_dependencies

COPY scripts/install_requirements.txt .

RUN pip install -r install_requirements.txt

ENV DORA_DEP_HOME /home/dora/workspace/dora_dependencies
# Install all the Python dependencies.
COPY scripts/install.sh  /home/dora/workspace/dora_dependencies/install.sh

# Get the Pylot models and code dependencies.
RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
RUN sudo apt-get install -y -q
RUN DEBIAN_FRONTEND=noninteractive sudo chmod +x /home/dora/workspace/dora_dependencies/install.sh
RUN DEBIAN_FRONTEND=noninteractive /home/dora/workspace/dora_dependencies/install.sh

ENV CARLA_HOME /home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1
# Clone scenario_runner.
RUN cd /home/dora/workspace && git clone https://github.com/carla-simulator/scenario_runner.git && cd scenario_runner && git checkout 0.9.10
# Install scenario_runner's dependencies.
RUN python3 -m pip install -r /home/dora/workspace/scenario_runner/requirements.txt
# Clone leaderboard.
RUN cd /home/dora/workspace && git clone https://github.com/carla-simulator/leaderboard.git && cd leaderboard && git checkout stable
RUN python3 -m pip install -r /home/dora/workspace/leaderboard/requirements.txt

RUN echo "export PYTHONPATH=/home/dora/workspace/dora_dependencies/dependencies/:/home/dora/workspace/dora_dependencies/dependencies/lanenet:/home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:/home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1/PythonAPI/carla/:/home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1/PythonAPI/carla/agents/:/home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1/PythonAPI/:/home/dora/workspace/scenario_runner:/home/dora/workspace/leaderboard" >> ~/.bashrc
RUN echo "export DORA_DEP_HOME=/home/dora/workspace/dora_dependencies" >> ~/.bashrc
# TODO: Remove $PYLOT_HOME Dependencies in Dora Dependencies
RUN echo "export PYLOT_HOME=/home/dora/workspace/dora_dependencies" >> ~/.bashrc

RUN echo "export CARLA_HOME=/home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1" >> ~/.bashrc
RUN echo "if [ -f ~/.bashrc ]; then . ~/.bashrc ; fi" >> ~/.bash_profile

# Set up ssh access to the container.
RUN cd ~/ && ssh-keygen -q -t rsa -N '' -f ~/.ssh/id_rsa <<<y 2>&1 >/dev/null
RUN sudo sed -i 's/#X11UseLocalhost yes/X11UseLocalhost no/g' /etc/ssh/sshd_config

WORKDIR /home/dora/workspace/dora_dependencies

WORKDIR /home/dora/workspace/dora-drives

RUN sudo wget https://dl.influxdata.com/telegraf/releases/telegraf-1.22.4_linux_amd64.tar.gz

RUN sudo tar xf telegraf-1.22.4_linux_amd64.tar.gz

COPY requirements.txt requirements.txt 

run python3 -m pip install -r requirements.txt

COPY . .

RUN python3 -m pip install /home/dora/workspace/dora-drives/wheels/dora-0.1.0-cp38-abi3-manylinux_2_31_x86_64.whl

RUN sudo chown dora:dora /home/dora/workspace/dora-drives

RUN sudo chmod +x /home/dora/workspace/dora-drives/scripts/*

