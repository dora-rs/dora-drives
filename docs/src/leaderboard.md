# Leaderboard

The OASIS Leaderboard enables you to test out your agent on predefined scenarios. It will generates KPI that can be used to compare agent between them. 

You can use the OASIS leaderboard without changing your graph by simply changing the input source. The new source is `carla/oasis_agent`.

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
    --env GRAPH=leaderboard/oasis_agent.yaml \
    --net=host \
    -it \
    --shm-size=2g \
    --memory=10g \
    --name dora \
    haixuantao/dora-drives
```

Or use the leaderboard natively:
```bash
dora-daemon --run-dataflow graphs/leaderboard/oasis_agent.yaml
```

To have a more fined-grained control over starting and stopping your agent,
you can use `start/stop` command from the cli:



```bash
# In a dedicated terminal
dora up

# In a client terminal
dora start graphs/leaderboard/oasis_agent.yaml

# ...
dora stop
```

You can name your dataflow as such:
```bash
# In a client terminal
dora start graphs/leaderboard/oasis_agent.yaml --name oasis-agent-demo

# ...
dora stop --name oasis-agent-demo
```

You will be able to find the results in the `graphs/leaderboard/` folder.