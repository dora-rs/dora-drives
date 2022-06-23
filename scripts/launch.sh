cd ../dora-rs
export RUSTFLAGS="--cfg tokio_unstable"
cargo build  --manifest-path coordinator/Cargo.toml --release # --features metrics # opentelemetry_jaeger 
cd ../dora-pylot
cargo build --release
cp ../dora-rs/target/release/dora-coordinator bin/dora-coordinator
cp target/release/dora-pylot-node bin/dora-pylot-node

# Carla start
nvidia-docker build --tag dora .
nvidia-docker run -itd --name dora -p 20022:22  dora /bin/bash
nvidia-docker exec -itd dora /home/dora/workspace/pylot/scripts/run_simulator.sh
nvidia-docker cp ~/.ssh/id_rsa.pub dora:/home/dora/.ssh/authorized_keys
nvidia-docker exec -i -t dora sudo chown dora /home/dora/.ssh/authorized_keys
nvidia-docker exec -i -t dora sudo service ssh start
ssh -p 20022 -X dora@localhost 
# nvidia-docker exec -it dora /home/dora/workspace/dora-rs/launch_in_container.sh