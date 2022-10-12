GRAPH=yolov5_dataflow.yaml

while getopts tbsg: flag
do
    case "${flag}" in
        b) build=1;;
        s) sim=1;;
        g) GRAPH="${OPTARG}";;
        t) tracing=1;;
    esac
done

# Replace existing dora container if it exist
[ "$(docker ps -a | grep dora)" ] && docker stop dora && docker rm dora 

if [ ! -z ${build+x} ]; then

    cd ../dora
    export RUSTFLAGS="--cfg tokio_unstable"
    cargo build  -p dora-coordinator --release  
    if [ ! -z ${tracing+x} ]; then
        cargo build  -p dora-runtime --release  --features tracing  
    else
        cargo build  -p dora-runtime --release 

    fi
    cd apis/python/node
    maturin build --release
    cd ../../../

    cd ../dora-drives
    mkdir -p bin
    cp ../dora/target/release/dora-coordinator bin/dora-coordinator
    cp ../dora/target/release/dora-runtime bin/dora-runtime
    find ../dora/target/release -type f -wholename "*/iceoryx-install/bin/iox-roudi" -exec cp {} bin \; 

    cp -r ../dora/target/wheels .
    docker build --tag haixuantao/dora-drives .
fi

# Add webcam if exist
if [ -e /dev/video0 ]; then
    echo "Webcam exists."
    docker run \
        --gpus all \
        --env-file variables.env \
        --device=/dev/video0:/dev/video0 \
        --net=host \
        -itd \
        --shm-size=256m \
        --name dora \
        haixuantao/dora-drives /bin/bash
        
    docker exec -itd dora sudo chown dora:dora /dev/video0
else
    echo "No webcam found."
    docker run \
        --gpus all \
        --env-file variables.env \
        --net=host \
        -itd \
        --shm-size=256m \
        --name dora \
        haixuantao/dora-drives /bin/bash
fi

if [ ! -z ${sim+x} ]; then
    docker exec -itd dora /home/dora/workspace/dora-drives/scripts/run_simulator.sh
fi


docker exec -itd dora /home/dora/workspace/dora-drives/bin/iox-roudi 
sleep 5
docker  exec -it dora /home/dora/workspace/dora-drives/bin/dora-coordinator run /home/dora/workspace/dora-drives/graphs/$GRAPH