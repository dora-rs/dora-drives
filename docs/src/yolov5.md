# Making the video stream intelligent

To add an operator, you will just have to specify it in the dataflow. Let's add a `yolov5` object detection operator that has already been written for us in `./operators/yolov5_op.py`.

```python
{{#include ../../operators/yolov5_op.py }}
```

The operator basically load a pytorch model and run it on `jpeg` encoded images. 
It sends the bounding box as a numpy bytes array with type `int32`.

To use it, just add it to the graph:

```yaml
{{#include ../../graphs/tutorials/webcam_yolov5.yaml }}
```

Inputs are prefixed by the node name to be able to separate name conflicts.

I've added capabilities for the plot to show the bounding box found by the `yolov5` operator in `physicals/plot.py`, which is basically mangling with cv2 API.

- To run it:

```bash
./scripts/launch.sh -b -g tutorials/webcam_yolov5.yaml
```

