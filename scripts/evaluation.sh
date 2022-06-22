export ROUTES=/home/erdos/workspace/leaderboard/data/routes_devtest.xml         # change to desired route
export SCENARIOS=/home/erdos/workspace/leaderboard/data/all_towns_traffic_scenarios_public.json
export TEAM_AGENT=/home/erdos/workspace/dora-rs/nodes/CarlaBaseAgent.py
export LEADERBOARD_ROOT=/home/erdos/workspace/leaderboard
export PORT=2000
export DEBUG_CHALLENGE=0
export REPETITIONS=1

python3 ${LEADERBOARD_ROOT}/leaderboard/leaderboard_evaluator.py \
--scenarios=${SCENARIOS}  \
--routes=${ROUTES} \
--repetitions=${REPETITIONS} \
--agent=${TEAM_AGENT} 