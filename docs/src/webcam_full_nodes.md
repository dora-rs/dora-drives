# Full perception

Let's add all `dora-drives` operators that works on image frame, which are:
- `yolov5` an object detector.
- `strong_sort` a multi-object tracker.
- `yolop` a lane and drivable area detector.
- `traffic_sign` a traffic sign detector.

the graph will look as follows:
```yaml
# graphs/tutorials/webcam_full.yaml

{{#include ../../graphs/tutorials/webcam_full.yaml}}
```

```bash
dora start graphs/tutorials/webcam_full.yaml --attach
```
> I'm currently having issue running all nodes behind the GFW. You can look into it for inspiration.

Nice 🥳 As you can see, the value of `dora` comes from the idea that you can compose different algorithm really quickly.