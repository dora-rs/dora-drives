#!/bin/bash

if [ -z $DORA_DEP_HOME ] ; then
    DORA_DEP_HOME=$(pwd)
    echo "WARNING: \$DORA_DEP_HOME is not set; Setting it to ${DORA_DEP_HOME}"
else
    echo "INFO: \$DORA_DEP_HOME is set to ${DORA_DEP_HOME}"
fi

# Install opencv separately because pip3 install doesn't install all libraries
# opencv requires.

#### . activate base
#### conda create -n dora3.7 python=3.8
#### conda activate dora3.7
#### pip install -r install_requirements.txt



###############################################################################
# Get models & code bases we depend on
###############################################################################
mkdir $DORA_DEP_HOME/dependencies
cd $DORA_DEP_HOME/dependencies/

#################### Download the code bases ####################
echo "[x] Compiling the planners..."
###### Build the FrenetOptimalTrajectory Planner ######
echo "[x] Compiling the Frenet Optimal Trajectory planner..."
cd $DORA_DEP_HOME/dependencies/
git clone https://github.com/erdos-project/frenet_optimal_trajectory_planner.git
cd frenet_optimal_trajectory_planner/
bash build.sh

###### Build the RRT* Planner ######
echo "[x] Compiling the RRT* planner..."
cd $DORA_DEP_HOME/dependencies/
git clone https://github.com/erdos-project/rrt_star_planner.git
cd rrt_star_planner/
bash build.sh

###### Build the Hybrid A* Planner ######
echo "[x] Compiling the Hybrid A* planner..."
cd $DORA_DEP_HOME/dependencies/
git clone https://github.com/erdos-project/hybrid_astar_planner.git
cd hybrid_astar_planner/
bash build.sh

###### Clone the Prediction Repository #####
echo "[x] Cloning the prediction code..."
cd $DORA_DEP_HOME/dependencies/
git clone https://github.com/erdos-project/prediction.git

. /opt/conda/etc/profile.d/conda.sh 
conda activate dora3.7 

cd $DORA_DEP_HOME/dependencies/

###### Install Carla Leaderboard ######
echo "[x] Installing Carla leaderboard..."
cd $DORA_DEP_HOME/dependencies/
# git clone  --single-branch https://github.com/carla-simulator/leaderboard.git
# python3 -m pip install -r leaderboard/requirements.txt

# git clone -b v0.9.13 --single-branch https://github.com/carla-simulator/scenario_runner.git
# python3 -m pip install -r scenario_runner/requirements.txt

git clone https://github.com/littlerants/Carsmos.git
python3 -m pip install -r Carsmos/requirements.txt


git clone --depth 1 https://github.com/ultralytics/yolov5
cd yolov5
wget https://github.com/ultralytics/yolov5/releases/download/v6.1/yolov5n.pt