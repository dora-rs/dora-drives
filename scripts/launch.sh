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
        --shm-size=2g \
        --memory=2g \
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
        --shm-size=2g \
        --memory=2g \
        --name dora \
        haixuantao/dora-drives /bin/bash
fi

if [ ! -z ${sim+x} ]; then
    nvidia-docker exec -itd dora /home/dora/workspace/dora-drives/scripts/run_simulator.sh
fi

if [ ! -z ${tracing+x} ]; then
    docker exec -itd dora /home/dora/workspace/dora-drives/telegraf-1.22.4/usr/bin/telegraf --config https://eu-central-1-1.aws.cloud2.influxdata.com/api/v2/telegrafs/09671055edbf6000
fi

sleep 5
nvidia-docker  exec -it dora /bin/bash -c "dora up && cd /home/dora/workspace/dora-drives && source /opt/conda/etc/profile.d/conda.sh && conda activate dora3.7 && dora-coordinator --run-dataflow graphs/$GRAPH"