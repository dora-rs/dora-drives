#!/bin/bash

if [ -z $DORA_DEP_HOME ] ; then
    DORA_DEP_HOME=$(pwd)
    echo "WARNING: \$DORA_DEP_HOME is not set; Setting it to ${DORA_DEP_HOME}"
else
    echo "INFO: \$DORA_DEP_HOME is set to ${DORA_DEP_HOME}"
fi

sudo apt-get -y update
sudo apt-get install -y git wget cmake unzip clang libpng-dev libgeos-dev

# Install opencv separately because pip3 install doesn't install all libraries
# opencv requires.

source activate base
pip install -r install_requirements.txt

sudo apt-get install -y python3-opencv
python3 -m pip install user gdown

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
cd $DORA_DEP_HOME/pylot/prediction/
git clone https://github.com/erdos-project/prediction.git

###### Download the Carla simulator ######
echo "[x] Downloading the CARLA 0.9.10.1 simulator..."
cd $DORA_DEP_HOME/dependencies/
if [ "$1" != 'challenge' ] && [ ! -d "CARLA_0.9.10.1" ]; then
    mkdir CARLA_0.9.10.1
    cd CARLA_0.9.10.1
    wget https://carla-releases.s3.eu-west-3.amazonaws.com/Linux/CARLA_0.9.10.1.tar.gz
    tar -xvf CARLA_0.9.10.1.tar.gz
    rm CARLA_0.9.10.1.tar.gz
fi