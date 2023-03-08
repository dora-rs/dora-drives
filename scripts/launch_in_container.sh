/home/dora/workspace/dora-drives/scripts/run_simulator.sh &
sleep 10 # wait for the simulator to get up
dora up # Up Iceoryx 
cd /home/dora/workspace/dora-drives
source /opt/conda/etc/profile.d/conda.sh
conda activate dora3.7
dora-daemon --run-dataflow graphs/$GRAPH