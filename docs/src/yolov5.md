# Making the video stream intelligent

Let's add a `yolov5` object detection operator that has already been written for us in `./operators/yolov5_op.py`. This will help us detect object as bounding boxes within the webcam stream.

```python
# operators/yolov5_op.py

{{#include ../../operators/yolov5_op.py }}
```

> Operators are composed of:
>
> `__init__` methods that help create the object.
>
> `on_event` methods that is called when an event is received. 
> There is currently 4 event types:
> - `STOP`: meaning that the operator was signalled to stop.
> - `INPUT`: meannig that an input was received.
>   - You can use `dora_event['id']`, to get the id. 
>   - You can use `dora_event['data']`, to get the data. 
>   - You can use `dora_event['meatadata']`, to get the metadata.
> - `INPUT_CLOSED`: meannig that an input source was closed. This could be useful if the input is critical for the well behaviour of the operator.
> - `ERROR`: meaning that error message was received.
> - `UNKNOWN`: meaning that an unknown message was received.
>
> We have encapsulated `input` event in a `on_input` method but this is not required.

To add an operator within the dataflow. You need to explicit what the input and output are. You can reference node by their ids:

```yaml
# graphs/tutorials/webcam_yolov5.yaml

{{#include ../../graphs/tutorials/webcam_yolov5.yaml }}
```

In this case, we have connected the `webcam/image` output to the `image` input of yolov5. `yolov5/bbox` is then connected to the `plot/obstacles_bbox`.

Inputs are prefixed by the node name to be able to separate name conflicts.

To run: 

```bash
dora up
dora start graphs/tutorials/webcam_yolov5.yaml --attach
```

> For more information on `yolov5`, go on [our `yolov5` detail page](./yolov5_operator.md)