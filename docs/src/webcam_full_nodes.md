# Full perception

Let's add all the operators currently provided by `dora-drives` that works on image frame. We currently have implemented:
- `yolov5` an object detector.
- `strong_sort` a multi-object tracker.
- `yolop` a lane and drivable area detector.
- `traffic_sign` a traffic sign detector.

the graph will look as follows:
```yaml
{{#include ../../graphs/tutorials/webcam_full.yaml}}
```

Run it with
```bash
./scripts/launch.sh -g tutorials/webcam_full.yaml
```

Nice 🥳 As you can see, the value of `dora` comes from the idea that you can compose different algorithm really quickly.