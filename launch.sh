cd ../dora-rs
export RUSTFLAGS="--cfg tokio_unstable"
cargo build  --manifest-path coordinator/Cargo.toml --release # --features metrics # opentelemetry_jaeger 
cargo build --manifest-path coordinator/Cargo.toml --examples --release # --features metrics # opentelemetry_jaeger 
cargo build --manifest-path runtime/Cargo.toml --release # --features metrics # opentelemetry_jaeger 
cd ../dora-pylot
cp ../dora-rs/target/release/dora-coordinator bin/dora-coordinator
cp ../dora-rs/target/release/examples/example_python_api bin/example_python_api
cp ../dora-rs/target/release/dora-runtime bin/dora-runtime

# Carla start
nvidia-docker build --tag dora .
nvidia-docker run -itd --name dora -p 20022:22  dora /bin/bash
nvidia-docker exec -itd dora /home/erdos/workspace/pylot/scripts/run_simulator.sh
nvidia-docker cp ~/.ssh/id_rsa.pub dora:/home/erdos/.ssh/authorized_keys
nvidia-docker exec -i -t dora sudo chown erdos /home/erdos/.ssh/authorized_keys
nvidia-docker exec -i -t dora sudo service ssh start
ssh -p 20022 -X erdos@localhost 
# nvidia-docker exec -it dora /home/erdos/workspace/dora-rs/launch_in_container.sh