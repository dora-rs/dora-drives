<p align="center">
    <img src="./logo.svg" width="400">
</p>

---

`dora-drives` is the starter kit for `dora`. `dora-drives` is an autonomous vehicle that runs within the [Carla simulator](https://carla.org/).

This project is in early development, and many features have yet to be implemented with breaking changes. Please don't take for granted the current design.

---

## Requirements:
|Dependencies|Version|
|-|-|
|OS| Linux
|Python| Version 3.8
|Docker|Latest


## Getting started

```bash
git clone git@github.com:futurewei-tech/dora.git
git clone git@github.com:futurewei-tech/dora-drives.git
cd dora-drives
```

- Run with:

```bash
docker run -d -p6831:6831/udp -p6832:6832/udp -p16686:16686 jaegertracing/all-in-one:latest
# TODO: Add Maturin build script
./scripts/launch.sh
```

And then within the container:
```bash
./workspace/dora-drives/scripts/launch_in_container.sh
```

### Configuration

- Current configurations are made as top-file constant variables. Later on, they will be integrated into the graph declaration.

#### InfluxDB visualisation

- Create a free plan at:  https://cloud2.influxdata.com/signup
- Create a bucket and an API token.
- Change the current static constant variables.

### Jaeger Export

- To export to Jaeger, you will need to first launch a Jaeger container:
```
docker run -d -p6831:6831/udp -p6832:6832/udp -p16686:16686 jaegertracing/all-in-one:latest
```

### Graph

```mermaid
flowchart TB
  carla_source_node[\carla_source_node/]
  perfect_detection_node
  obstacles_location_node
  planning_node
  pid_control_node
  control_node
  influxdb_node[/influxdb_node\]
  carla_source_node -- depth_frame --> perfect_detection_node
  carla_source_node -- position --> perfect_detection_node
  carla_source_node -- segmented_frame --> perfect_detection_node
  carla_source_node -- depth_frame --> obstacles_location_node
  perfect_detection_node -- obstacles_without_location --> obstacles_location_node
  carla_source_node -- position --> obstacles_location_node
  obstacles_location_node -- obstacles --> planning_node
  carla_source_node -- position --> planning_node
  carla_source_node -- position --> pid_control_node
  planning_node -- waypoints --> pid_control_node
  pid_control_node -- control --> control_node
  carla_source_node -- vehicle_id --> control_node
  control_node -- control_status --> influxdb_node
  obstacles_location_node -- obstacles --> influxdb_node
  carla_source_node -- position --> influxdb_node

```