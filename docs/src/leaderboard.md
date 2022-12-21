# Leaderboard

The leaderboard graphs connect dora to the leaderboard 2.0 to run the graph within the leaderboard environment.

To use the leaderboard through docker
```bash
./scripts/launch.sh -b -s -g leaderboard/yolov5_agent.yaml
```

To use the leaderboard natively:
```bash
dora-coordinator --run-dataflow graphs/leaderboard/yolov5_agent.yaml
```
