# Getting started

This first tutorial enables to stream a video stream from a webcam from scratch.

1. Fork this project

```bash
# Go to: https://github.com/dora-rs/dora-drives/fork
#
# Then clone your fork:
git clone git@github.com:<USERNAME>/dora-drives.git

# Add dora as a remote source to be able to fetch updates.
git remote add dora git@github.com:dora-rs/dora-drives.git
```

You will find the following folder structure
```bash
.
├── graphs # Example graph
├── operators # Exemple operators
├── carla # Carla nodes and operators that requires the CARLA API
├── ros # ROS based operators to bridge between ROS and dora
├── docs # This docs folder. You can replace the src file to keep your operator documented.
├── ... # utils folder
```

2. To be able to run dora, you will need to start `dora-coordinator` and `dora-daemon`:
```bash
# Start the `dora-coordinator` and `dora-daemon`. 
dora up 
```

3. To start a dataflow, you just need to pass a dataflow path.
```bash
conda activate dora3.7
dora start graphs/tutorials/webcam.yaml --attach --hot-reload --name webcam
```

> `--attach`: enables you to wait for the dataflow to finish 
> before returning.
> 
> `--hot-reload`: enables you to modify Python Operator while the 
> dataflow is running.
>
> `--name`: enables you to name a dataflow that might be simpler to use than the UUID.

4. You should see a small webcam open up!
> Make sure to have a webcam and cv2 install via: `pip install opencv-python`

5. To stop your dataflow, you can use <kbd>ctrl</kbd>+<kbd>c</bkbd>

6. That's it! You know the basic of dora!