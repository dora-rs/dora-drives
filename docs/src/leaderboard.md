# Leaderboard

The CARLA Leaderboard enables you to test out your agent on predefined scenarios. It will generates KPI that can be used to compare agent between them. 

You can use the CARLA leaderboard without changing your graph by simply changing the input source. The new source is `carla/dora_agent`.

The leaderboard graphs connect dora to the leaderboard 2.0 to run the graph within the leaderboard environment.

To use the leaderboard through docker, first build it with:
```bash
docker build . -t haixuantao/dora-drives
```

or pull it from dockerhub:
```bash
docker pull haixuantao/dora-drives
```

Then run it within docker with:

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

Or use the leaderboard natively:
```bash
dora-coordinator --run-dataflow graphs/leaderboard/full_agent.yaml
```


To download the results from the docker container, you can use:
```bash
## For the camera feed
docker cp dora:/home/dora/workspace/dora-drives/graphs/leaderboard/output.avi .
## For the leaderboard `simulation_results.json` use:
docker cp dora:/home/dora/workspace/dora-drives/graphs/leaderboard/simulation_results.json .
```

Otherwise you will be able to find the results in the `graphs/leaderboard/` folder.