# Installation

## From Docker Hub

### Requirements

You will only need [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

To start with docker which is the easiest:
```bash
docker pull haixuantao/dora-drives
nvidia-docker run -itd --name dora -p 20022:22 haixuantao/dora-drives /bin/bash
nvidia-docker exec -itd dora /home/dora/workspace/dora-drives/scripts/run_simulator.sh
nvidia-docker cp ~/.ssh/id_rsa.pub dora:/home/dora/.ssh/authorized_keys
nvidia-docker exec -i -t dora sudo chown dora /home/dora/.ssh/authorized_keys
nvidia-docker exec -i -t dora sudo service ssh start
ssh -p 20022 -X dora@localhost 
```

And then within the container:
```bash
./workspace/dora-drives/scripts/launch_in_container.sh
```

> This docker image has been built with my setup and it might not work on all machines. In case it doesn't work. Please check the following `From Source`.

## From Source

### Requirements

You will need:
- [Rust](https://rustup.rs/)
- [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- Python v3.8.10
- maturin `pip install maturin`

For linux, run:
```bash
git clone git@github.com:futurewei-tech/dora.git
git clone git@github.com:futurewei-tech/dora-drives.git
cd dora-drives
./scripts/launch.sh
```

> For other OS, you will need to adapt the launch.sh script.

And then within the container:
```bash
./workspace/dora-drives/scripts/launch_in_container.sh
```

> This script has been built with my setup and you might need to install further dependencies that I have not listed, and additional configration for cross-compiling.

### Hardware requirements

- NVIDIA GPU Card
- x86 Intel CPU (preferable)
- Linux