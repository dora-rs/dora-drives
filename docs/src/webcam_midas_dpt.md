# Depth estimation

MiDaS models for computing relative depth from a single image.

> MiDaS computes relative inverse depth from a single image. The repository provides multiple models that cover different use cases ranging from a small, high-speed model to a very large model that provide the highest accuracy. The models have been trained on 10 distinct datasets using multi-objective optimization to ensure high quality on a wide range of inputs.
Let's add all `dora-drives` operators that works on image frame, which are:
- `webcam` plug in a camera.
- `single_dpt` computing relative depth from a single image.
- `plot` take almost all output from the graph and plot it on the camera frame.

the graph will look as follows:
```yaml
# graphs/tutorials/webcam_single_dpt_frame.yaml

{{#include ../../graphs/tutorials/webcam_single_dpt_frame.yaml}}
```

In this case, we have connected the `webcam/image` output to the `image` input of single_dpt. `single_dpt/depth_frame` is then connected to the `plot/image`.

Inputs are prefixed by the node name to be able to separate name conflicts.

To run: 

```bash
dora up
dora start graphs/tutorials/webcam_single_dpt_frame.yaml --attach --hot-reload --name dpt_midas
```

Display as follows:
<p align="center">
    <img src="./midas_dpt.png.png" width="800">
</p>

> I'm currently having issue running all nodes behind the GFW. You can look into it for inspiration.

Nice 🥳 As you can see, the value of `dora` comes from the idea that you can compose different algorithm really quickly.