FROM nvidia/cudagl:11.4.2-devel-ubuntu20.04

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
RUN sudo DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata

# Install conda

# Get the dora package dependencies.
RUN sudo apt-get -y install apt-utils git curl clang wget
# Install miniconda
RUN echo "export CONDA_DIR=/opt/conda" >> ~/.bashrc
RUN echo "export PATH=$CONDA_DIR/bin:$PATH" >> ~/.bashrc
RUN sudo wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda.sh && \
    sudo /bin/bash ~/miniconda.sh -b -p /opt/conda
ENV CONDA_DIR /opt/conda
ENV PATH $CONDA_DIR/bin:$PATH 
RUN sudo chown -R dora:dora /opt/conda
RUN conda init bash

# Setup Rust and install dora.
RUN mkdir -p /home/dora/workspace

RUN sudo apt-get -y --fix-missing update && sudo apt-get install --fix-missing -y libcudnn8 ssh libqt5core5a libeigen3-dev cmake qtbase5-dev libpng16-16 libtiff5 python3-tk libgeos-dev vim build-essential libopenblas-dev

RUN mkdir -p /home/dora/workspace/dora_dependencies

WORKDIR /home/dora/workspace/dora_dependencies

COPY scripts/install_requirements.txt .

ENV DORA_DEP_HOME /home/dora/workspace/dora_dependencies
# Install all the Python dependencies.
COPY scripts/install.sh  /home/dora/workspace/dora_dependencies/install.sh

# Get the Pylot models and code dependencies.
RUN echo 'debconf debconf/frontend select Noninteractive' | sudo debconf-set-selections
RUN sudo apt-get install -y -q
RUN DEBIAN_FRONTEND=noninteractive sudo chmod +x /home/dora/workspace/dora_dependencies/install.sh
RUN DEBIAN_FRONTEND=noninteractive /home/dora/workspace/dora_dependencies/install.sh

ENV CARLA_HOME /home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1
# # Clone scenario_runner.
# RUN cd /home/dora/workspace && git clone https://github.com/carla-simulator/scenario_runner.git && cd scenario_runner && git checkout 0.9.10
# # Install scenario_runner's dependencies.
# RUN python3 -m pip install -r /home/dora/workspace/scenario_runner/requirements.txt
# # Clone leaderboard.
# RUN cd /home/dora/workspace && git clone https://github.com/carla-simulator/leaderboard.git && cd leaderboard && git checkout stable
# RUN python3 -m pip install -r /home/dora/workspace/leaderboard/requirements.txt

RUN echo "if [ -f ~/.bashrc ]; then . ~/.bashrc ; fi" >> ~/.bash_profile
# Set up ssh access to the container.
RUN cd ~/ && ssh-keygen -q -t rsa -N '' -f ~/.ssh/id_rsa
RUN sudo sed -i 's/#X11UseLocalhost yes/X11UseLocalhost no/g' /etc/ssh/sshd_config

WORKDIR /home/dora/workspace/dora_dependencies

WORKDIR /home/dora/workspace/dora-drives

# RUN sudo wget https://dl.influxdata.com/telegraf/releases/telegraf-1.22.4_linux_amd64.tar.gz

# RUN sudo tar xf telegraf-1.22.4_linux_amd64.tar.gz

RUN conda activate dora3.8 && pip install --upgrade pip
RUN conda activate dora3.8 && conda install pytorch=1.11.0 torchvision cudatoolkit=11.3 -c pytorch 
RUN conda activate dora3.8 && python3 -c "import torch; assert torch.cuda.is_available(), 'CUDA seems to not be available on build, Check out :https://github.com/pytorch/extension-cpp/issues/71#issuecomment-1061880626'" 

RUN conda activate dora3.8 && MAX_JOBS=4 python3 -m pip install -U git+https://github.com/NVIDIA/MinkowskiEngine -v --no-deps --install-option="--blas_include_dirs=${CONDA_PREFIX}/include" --install-option="--blas=openblas" --install-option="--force_cuda"

COPY requirements.txt requirements.txt 
RUN conda activate dora3.8 && python3 -m pip install -r requirements.txt

RUN sudo chown -R dora:dora /home/dora

# Cache model weight
RUN conda activate dora3.8 && python3 -c "from imfnet import get_model; get_model()"
RUN conda activate dora3.8 && python3 -c "import torch; torch.hub.load('ultralytics/yolov5', 'yolov5n')"
RUN conda activate dora3.8 && python3 -c "import torch; torch.hub.load('hustvl/yolop', 'yolop', pretrained=True)"
RUN conda activate dora3.8 && python3 -c "from strong_sort import StrongSORT; import torch; StrongSORT('osnet_x0_25_msmt17.pt', torch.device('cuda'), False)"
RUN conda activate dora3.8 && python3 -c "import yolov7_tt100k"

COPY . .

COPY .bashrc  /home/dora/.bashrc

RUN conda activate dora3.8 && python3 -m pip install /home/dora/workspace/dora-drives/wheels/dora-0.1.0-cp38-abi3-manylinux_2_31_x86_64.whl

RUN sudo chmod +x /home/dora/workspace/dora-drives/carla/carla_source_node.py
RUN sudo chmod +x /home/dora/workspace/dora-drives/scripts/*
