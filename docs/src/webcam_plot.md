## Getting started

This first tutorial enables to stream a video stream from a webcam from scratch.

1. Create a new dataflow

```bash
dora new abc_project --lang python
cd abc_project
```

This creates the following `abc_project` directory
```bash
.
├── dataflow.yml
├── node_1
│   └── node_1.py
├── op_1
│   └── op_1.py
└── op_2
    └── op_2.py
```

3. Start `dora-daemon` in a separate terminal window
```bash
# New terminal window
dora up 
```

4. Start your dataflow
```bash
# Other window
dora start dataflow.yml
# Output: c95d118b-cded-4531-a0e4-cd85b7c3916c
```
The output is the unique ID of the dataflow instance, which can be used to control it through the `dora` CLI.

5. You will see in your `dora-daemon` window operators receiving ticks.
```bash
Received input tick, with data: b''
Received input tick, with data: b''
Received input tick, with data: b''
...
```

6. Stop your dataflow
```bash
dora stop c95d118b-cded-4531-a0e4-cd85b7c3916c
```
(Pass the ID returned by `dora start` here.)

7. You can then add or modify operators or nodes. For adding nodes easily, you can use the `dora` CLI again:

- Run `dora new --kind operator --lang rust <name>` to create a new Rust operator named `<name>`.
- Run `dora new --kind custom-node --lang rust <name>` to create a new custom node named `<name>`.

You need to add the created operators/nodes to your dataflow YAML file.

8. You can also download already implemented operators by putting links in the dataflow. This example will launch a webcam plot stream. 

```yaml
communication:
  zenoh:
    prefix: /abc_project

nodes:
  - id: op_1
    operator:
      python: https://raw.githubusercontent.com/dora-rs/dora-drives/main/operators/webcam_op.py
      inputs:
        tick: dora/timer/millis/100
      outputs:
        - image
  - id: op_2
    operator:
      python: https://raw.githubusercontent.com/dora-rs/dora-drives/main/physicals/plot.py
      inputs:
        image: op_1/image 
```

9. Then restart the dataflow

```bash
dora start dataflow.yml
```

> Make sure to have a webcam and cv2 install via: `pip install opencv-python`

You should see a small webcam open up!
