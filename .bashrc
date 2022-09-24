export CONDA_DIR=/opt/conda
export PATH=/bin:/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

# >>> conda initialize >>>
# !! Contents within this block are managed by 'conda init' !!
__conda_setup="$('/opt/conda/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
        . "/opt/conda/etc/profile.d/conda.sh"
    else
        export PATH="/opt/conda/bin:$PATH"
    fi
fi
unset __conda_setup
# <<< conda initialize <<<

conda activate dora3.8

export PYTHONPATH=/home/dora/workspace/dora_dependencies/dependencies/:/home/dora/workspace/dora_dependencies/dependencies/lanenet:/home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg:/home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1/PythonAPI/carla/:/home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1/PythonAPI/carla/agents/:/home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1/PythonAPI/:/home/dora/workspace/scenario_runner:/home/dora/workspace/leaderboard:/home/dora/workspace/dora-drives/carla:/home/dora/workspace/dora-drives/operators
export PATH=/opt/conda/envs/dora3.8/bin:/opt/conda/condabin:/bin:/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/home/dora/workspace/dora-drives/bin/

export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/lib/i386-linux-gnu:/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/usr/local/nvidia/lib:/usr/local/nvidia/lib64:/opt/conda/envs/dora3.8/lib/

export CARLA_HOME=/home/dora/workspace/dora_dependencies/dependencies/CARLA_0.9.10.1
export PYLOT_HOME=/home/dora/workspace/dora_dependencies

export DORA_DEP_HOME=/home/dora/workspace/dora_dependencies
export DORA_RUNTIME_PATH=/home/dora/workspace/dora-drives/bin/dora-runtime
export DORA_PATH=/home/dora/workspace/dora-drives