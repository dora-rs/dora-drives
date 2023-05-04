# Depth estimation example

MiDaS models for computing relative depth from a single image.

> MiDaS computes relative inverse depth from a single image. The repository provides multiple models that cover different use cases ranging from a small, high-speed model to a very large model that provide the highest accuracy. The models have been trained on 10 distinct datasets using multi-objective optimization to ensure high quality on a wide range of inputs.
Let's add all `dora-drives` operators that works on image frame, which are:
- `webcam` plug in a camera.
- `midas_op` computing relative depth from a single image.
- `plot` take almost all output from the graph and plot it on the camera frame.

the graph will look as follows:
```yaml
# graphs/tutorials/webcam_midas_frame.yaml

{{#include ../../graphs/tutorials/webcam_midas_frame.yaml}}
```

### MiDaS source code:
You need to link the network to load the algorithm model.
If you are in mainland China, you may need a network proxy to speed up the download.

optional:
    Of course, you can also download it in advance and place it under the specified directory. The operation steps are as follows:
```bash
cd $DORA_DEP_HOME/dependencies/
git clone git@github.com:isl-org/MiDaS.git
cd MiDaS/weights
# If you don't want to add manual download, the program will also automatically download the model file
wget https://github.com/isl-org/MiDaS/releases/download/v2_1/midas_v21_small_256.pt
cp midas_v21_small_256.pt $HOME/.cache/torch/hub/checkpoints/
```
At the same time, open the following comments in the dataflow configuration file # graphs/tutorials/webcam_midas_frame.yaml
```yaml
      MIDAS_PATH: $DORA_DEP_HOME/dependencies/MiDaS/
      MIDAS_WEIGHT_PATH: $DORA_DEP_HOME/dependencies/MiDaS/weights/midas_v21_small_256.pt
      MODEL_TYPE: "MiDaS_small"
      MODEL_NAME: "MiDaS_small"
```
- model_type = "DPT_Large"     # MiDaS v3 - Large     (highest accuracy, slowest inference speed)
- model_type = "DPT_Hybrid"   # MiDaS v3 - Hybrid    (medium accuracy, medium inference speed)
- model_type = "MiDaS_small"  # MiDaS v2.1 - Small   (lowest accuracy, highest inference speed)

### Custom configuration

1) Pick one or more models, You can [find out more here](https://github.com/isl-org/MiDaS/#setup).

2) Other descriptive information
In this case, we have connected the `webcam/image` output to the `image` input of midas_op. `midas_op/depth_frame` is then connected to the `plot/image`.
Inputs are prefixed by the node name to be able to separate name conflicts.

### To run: 

```bash
dora up
dora start graphs/tutorials/webcam_midas_frame.yaml --attach --hot-reload --name dpt_midas
```

Display as follows:
<p align="center">
    <img src="./midas_dpt.png" width="500">
</p>

> I'm currently having issue running all nodes behind the GFW. You can look into it for inspiration.

Nice 🥳 As you can see, the value of `dora` comes from the idea that you can compose different algorithm really quickly.