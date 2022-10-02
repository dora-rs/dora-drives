<p align="center">
    <img src="./docs/src/logo.svg" width="400">
</p>

---

`dora-drives` is a set of operators you can use with `dora` to create an autonomous driving vehicle.

You can test the operators on real webcam or within [Carla](https://carla.org/).

This project is in early development, and many features have yet to be implemented with breaking changes. Please don't take for granted the current design.

## Documentation

The documentation can be found here: [dora-rs.github.io/dora-drives](https://dora-rs.github.io/dora-drives)

You will be able to get started using the [installation section](https://dora-rs.github.io/dora-drives/installation.html).

## Operators:

### [Point cloud registration](https://paperswithcode.com/task/point-cloud-registration/latest)

- [IMFNet](https://github.com/XiaoshuiHuang/IMFNet) 

[![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/imfnet-interpretable-multimodal-fusion-for/point-cloud-registration-on-3dmatch-benchmark)](https://paperswithcode.com/sota/point-cloud-registration-on-3dmatch-benchmark?p=imfnet-interpretable-multimodal-fusion-for)

<img src=https://user-images.githubusercontent.com/22787340/192553644-1d8466be-c1e1-492d-9bc2-3546a1a3ea55.png width="300"><img src=https://user-images.githubusercontent.com/22787340/192553631-d6379df5-a1a8-49b9-a5c1-7bc7b9f1ed52.png width="300">



### [Object dectection](https://paperswithcode.com/task/object-detection)

- [yolov5](https://github.com/ultralytics/yolov5) 

[![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/path-aggregation-network-for-instance/object-detection-on-coco)](https://paperswithcode.com/sota/object-detection-on-coco?p=path-aggregation-network-for-instance)

<img src=https://user-images.githubusercontent.com/22787340/187723794-3623bee2-91d6-436a-a5d7-d2e363483c76.gif width="600">

- Perfect detection on Carla Simulator

### [Traffic sign recognition](https://paperswithcode.com/task/traffic-sign-recognition)
    
- [Custom trained yolov7 on tt100k](https://github.com/haixuanTao/yolov7)

### [Lane detection](https://paperswithcode.com/task/lane-detection)

- [yolop](https://github.com/hustvl/YOLOP) 

[![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/hybridnets-end-to-end-perception-network-1/lane-detection-on-bdd100k)](https://paperswithcode.com/sota/lane-detection-on-bdd100k?p=hybridnets-end-to-end-perception-network-1) 

<img src=https://user-images.githubusercontent.com/22787340/187723841-7f3ba560-dbbe-4d43-886a-fb3b0be9247a.gif width="600">

### [Drivable Area detection](https://paperswithcode.com/task/drivable-area-detection)

- [yolop](https://github.com/hustvl/YOLOP) 

[![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/hybridnets-end-to-end-perception-network-1/drivable-area-detection-on-bdd100k)](https://paperswithcode.com/sota/drivable-area-detection-on-bdd100k?p=hybridnets-end-to-end-perception-network-1) 

### [Multiple Object tracking(MOT)](https://paperswithcode.com/task/multi-object-tracking)
#### [strong sort](https://github.com/haixuanTao/yolov5_strongsort_package) 

[![PWC](https://img.shields.io/endpoint.svg?url=https://paperswithcode.com/badge/strongsort-make-deepsort-great-again/multi-object-tracking-on-mot20-1)](https://paperswithcode.com/sota/multi-object-tracking-on-mot20-1?p=strongsort-make-deepsort-great-again) 

<img src=https://user-images.githubusercontent.com/22787340/187723873-473cda4f-573d-4663-a5b9-a4df2611c482.gif width="600">

### [Motion Planning](https://paperswithcode.com/task/motion-planning)

- [Hybrid A-star](https://github.com/erdos-project/hybrid_astar_planner)

<img src=https://user-images.githubusercontent.com/22787340/192555777-1d5e4c5f-d654-4ef3-a019-387e56e46970.gif width="600">

### Path Tracking

- Proportional Integral Derivative controller (PID)
    
## Future operators:

- [Trajectory Prediction (pedestrian and vehicles)](https://paperswithcode.com/task/trajectory-prediction)

- [Pedestrian detection](https://paperswithcode.com/task/pedestrian-detection)

- [Semantic segmentation](https://paperswithcode.com/task/semantic-segmentation)

- [Depth estimation](https://paperswithcode.com/task/depth-estimation)

- [Multiple object tracking and segmentation(MOTS)](https://paperswithcode.com/task/multi-object-tracking)

## ⚖️ LICENSE 

This project is licensed under Apache-2.0. Check out [NOTICE.md](NOTICE.md) for more information.
