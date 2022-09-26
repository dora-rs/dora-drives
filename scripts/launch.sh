# cd ../dora
# export RUSTFLAGS="--cfg tokio_unstable"
# cargo build  --manifest-path binaries/coordinator/Cargo.toml --release # --features metrics # opentelemetry_jaeger 
# cargo build  --manifest-path binaries/runtime/Cargo.toml --release # --features metrics # opentelemetry_jaeger 
# cd apis/python/node
# maturin build --release
# cd ../../../

# cd ../dora-drives
# mkdir -p bin
# cp ../dora/target/release/dora-coordinator bin/dora-coordinator
# cp ../dora/target/release/dora-runtime bin/dora-runtime
# find ../dora/target/release -type f -wholename "*/iceoryx-install/bin/iox-roudi" -exec cp {} bin \; 

# cp -r ../dora/target/wheels .

# ssh access
# nvidia-docker cp ~/.ssh/id_rsa.pub dora:/home/dora/.ssh/authorized_keys
# nvidia-docker exec -i -t dora sudo chown dora /home/dora/.ssh/authorized_keys
# nvidia-docker exec -i -t dora sudo service ssh start

# Carla start
docker build --tag haixuantao/dora-drives .
# Replace existing dora container if it exist
[ "$(docker ps -a | grep dora)" ] && docker stop dora && docker rm dora 
docker run --gpus all --env-file variables.env --net=host -e DISPLAY -itd --shm-size=256m --name dora haixuantao/dora-drives /home/dora/workspace/dora-drives/scripts/run_simulator.sh
docker exec -itd dora /home/dora/workspace/dora-drives/bin/iox-roudi 
sleep 5
docker  exec -it dora /home/dora/workspace/dora-drives/bin/dora-coordinator run /home/dora/workspace/dora-drives/graphs/yolov5_dataflow.yaml