<p align="center">
    <img src="./docs/src/logo.svg" width="400">
</p>

---

`dora-drives` is the starter kit for `dora`. `dora-drives` is an autonomous vehicle that runs within the [Carla simulator](https://carla.org/).

This project is in early development, and many features have yet to be implemented with breaking changes. Please don't take for granted the current design.

---

### Operator category already implemented

- [Object dectection](https://paperswithcode.com/task/object-detection)
    - Perfect detection
    - [yolov5](https://github.com/ultralytics/yolov5) [![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/path-aggregation-network-for-instance/object-detection-on-coco)](https://paperswithcode.com/sota/object-detection-on-coco?p=path-aggregation-network-for-instance)
- [Lane detection](https://paperswithcode.com/task/lane-detection)
    - [yolop](https://github.com/hustvl/YOLOP) [![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/hybridnets-end-to-end-perception-network-1/lane-detection-on-bdd100k)](https://paperswithcode.com/sota/lane-detection-on-bdd100k?p=hybridnets-end-to-end-perception-network-1) 
- [Drivable Area detection](https://paperswithcode.com/task/drivable-area-detection)
    - [yolop](https://github.com/hustvl/YOLOP) [![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/hybridnets-end-to-end-perception-network-1/drivable-area-detection-on-bdd100k)](https://paperswithcode.com/sota/drivable-area-detection-on-bdd100k?p=hybridnets-end-to-end-perception-network-1) 
- [Motion Planning](https://paperswithcode.com/task/motion-planning)
    - Hybrid A-star.
- Path Tracking
    - PID

### Future operator categories with high priority:

- [Traffic sign recognition](https://paperswithcode.com/task/traffic-sign-recognition)
    - Currently falling back to yolov5. 
- [Motion Planning](https://paperswithcode.com/task/motion-planning)
    - Better motion planning. 
- [Trajectory Prediction (pedestrian and vehicles)](https://paperswithcode.com/task/trajectory-prediction)
    - Currently unimplemented. 
- [Multiple Object tracking(MOT)](https://paperswithcode.com/task/multi-object-tracking)
    - Currently unimplemented. We will probably go with Strong sort as yolov5+StrongSort seems to have better performance. See: https://github.com/mikel-brostrom/Yolov5_StrongSORT_OSNet [![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/strongsort-make-deepsort-great-again/multi-object-tracking-on-mot20-1)](https://paperswithcode.com/sota/multi-object-tracking-on-mot20-1?p=strongsort-make-deepsort-great-again) 
    
### Future operator categories with less priority:
- [Pedestrian detection](https://paperswithcode.com/task/pedestrian-detection)
    - Fall back to yolov5 currently
- [Semantic segmentation](https://paperswithcode.com/task/semantic-segmentation)
    - Current: fall back to drivable area only. 
- [Depth estimation](https://paperswithcode.com/task/depth-estimation)
    - Current: fall back to depth frame and lidar sensor. 
- [Multiple object tracking and segmentation(MOTS)](https://paperswithcode.com/task/multi-object-tracking)
    - Current: Fall back to object detection + Object tracking. 

---
## Documentation

The documentation can be found here: https://dora-rs.github.io/dora-drives

## Docker started

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

> This docker image has been built with my setup and it might not work on all machines. In case it doesn't work. Please check the following `Getting Started`.

## Getting started

```bash
git clone git@github.com:futurewei-tech/dora.git
git clone git@github.com:futurewei-tech/dora-drives.git
cd dora-drives
```

- Run with:

```bash
docker run -d -p6831:6831/udp -p6832:6832/udp -p16686:16686 jaegertracing/all-in-one:latest
# TODO: Add Maturin build script
./scripts/launch.sh
```

> Make sure to kill running dora containers if you are trying to build a new image via
> ```bash
> docker stop dora && docker rm dora
> ```

And then within the container:
```bash
./workspace/dora-drives/scripts/launch_in_container.sh
```

### Configuration

- Current configurations are made as top-file constant variables. Later on, they will be integrated into the graph declaration.
