
# Define the desired version of pip
desired_version="0.2.6"

# Check if pip is installed and get its version
if command -v dora >/dev/null 2>&1; then
    installed_version=$(dora --version | awk '{print $2}')
    if [ "$installed_version" = "$desired_version" ]; then
        echo "dora version $desired_version is installed"
    else
        echo "dora version $installed_version is installed (expected $desired_version)"
	DORA_VERSION=v0.2.6 # Check for the latest release
	ARCHITECTURE=$(uname -m)
	wget https://github.com/dora-rs/dora/releases/download/${DORA_VERSION}/dora-${DORA_VERSION}-${ARCHITECTURE}-Linux.zip
	sudo unzip -o -q dora-${DORA_VERSION}-${ARCHITECTURE}-Linux.zip -d /bin/
	pip install dora-rs==${DORA_VERSION} ## For Python API

    ## Update frenet
    pip install git+https://github.com/haixuanTao/frenet_optimal_trajectory_planner.git

    fi
else
    echo "dora is not installed"
    DORA_VERSION=v0.2.6 # Check for the latest release
    ARCHITECTURE=$(uname -m)
    sudo wget https://github.com/dora-rs/dora/releases/download/${DORA_VERSION}/dora-${DORA_VERSION}-${ARCHITECTURE}-Linux.zip
    sudo unzip -o -q dora-${DORA_VERSION}-${ARCHITECTURE}-Linux.zip -d /bin/
    pip install dora-rs==${DORA_VERSION} ## For Python API
    
    ## Update frenet
    pip install git+https://github.com/haixuanTao/frenet_optimal_trajectory_planner.git
fi
