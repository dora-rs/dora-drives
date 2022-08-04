cd ../dora
export RUSTFLAGS="--cfg tokio_unstable"
cargo build  --manifest-path binaries/coordinator/Cargo.toml --release # --features metrics # opentelemetry_jaeger 
cargo build  --manifest-path binaries/runtime/Cargo.toml --release # --features metrics # opentelemetry_jaeger 
cd ../dora-drives
cp ../dora/target/release/dora-coordinator bin/dora-coordinator
cp ../dora/target/release/dora-runtime bin/dora-runtime
cp -r ../dora/target/wheels .


# Carla start
nvidia-docker build --tag dora .
nvidia-docker run -itd --name dora -p 20022:22  dora /bin/bash
nvidia-docker exec -itd dora /home/dora/workspace/dora-drives/scripts/run_simulator.sh
nvidia-docker cp ~/.ssh/id_rsa.pub dora:/home/dora/.ssh/authorized_keys
nvidia-docker exec -i -t dora sudo chown dora /home/dora/.ssh/authorized_keys
nvidia-docker exec -i -t dora sudo service ssh start
ssh -p 20022 -X dora@localhost 
# nvidia-docker exec -it dora /home/dora/workspace/dora-drives/launch_in_container.sh