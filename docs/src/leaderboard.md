# Leaderboard

The leaderboard graphs connect dora to the leaderboard 2.0 to run the graph within the leaderboard environment.

To use the leaderboard through docker, first build it with:
```bash
docker build . -t haixuantao/dora-drives
```

or pull it from dockerhub:
```bash
docker pull haixuantao/dora-drives
```

```bash
docker run \
    --gpus all \
    --runtime=nvidia \
    --env-file variables.env \
    --env GRAPH=leaderboard/full_agent.yaml \
    --net=host \
    -it \
    --shm-size=2g \
    --memory=10g \
    --name dora \
    haixuantao/dora-drives
```

To use the leaderboard natively:
```bash
dora-coordinator --run-dataflow graphs/leaderboard/yolov5_agent.yaml
```
