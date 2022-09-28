# Streaming live video on your monitor

The principle of `dora` is to declare a dataflow of nodes that are linked with a publish-subscribe service. 

The nodes can be either:
- Custom nodes: Nodes that uses `dora` node API which gives you a way to receive and send messages through the dataflow.
- Runtime nodes: Nodes that are managed by a `dora-runtime`. They requires that applications are written as `operators` following a specific framework. This operators will have more features than the `dora` node API.  

## Getting started

Let's write a small webcam streaming application.

- First, we're going to write a webcam node, that is going to capture the webcam image each time it receive an input and send it as a jpeg into the `image` pubsub service.

```python
{{#include ../../physicals/webcam.py }}
```

- Then, we're going to write a plot operator that is going to be managed by `dora-runtime`. It's going to decode the jpeg and plot it as a window.

```python
{{#include ../../physicals/simple_plot.py }}
```

- To finish, we're going to declare the graph for `dora-coordinator` to know how to link those nodes together. In this exemple we're going to use `Iceoryx` as the pubsub provider:

```yaml
{{#include ../../graphs/tutorials/webcam_dataflow.yaml }}
```

> dora timer input is a standard input that tick at the configured frequency.

- We can now run this pipeline using a `launch` script that I wrote, that compile `dora` and its API, put them in a docker and run the dataflow. 

```bash
./scripts/launch.sh -b -g tutorials/webcam_dataflow.yaml
```

> You will need a webcam to run this tutorial.