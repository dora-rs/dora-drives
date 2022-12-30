/home/dora/workspace/dora-drives/scripts/run_simulator.sh &
python ${CARLA_HOME}/PythonAPI/util/config.py -m Town12
dora up
cd /home/dora/workspace/dora-drives
source /opt/conda/etc/profile.d/conda.sh
conda activate dora3.7
dora-coordinator --run-dataflow graphs/$GRAPH