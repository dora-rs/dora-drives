use dora_node_api::{self, config::DataId, DoraNode};

use futures::StreamExt;
use std::{
    collections::BTreeMap,
    time::{Duration, Instant},
};
use tokio::time::sleep;

static ATOMIC_TIMEOUT: Duration = Duration::from_millis(200);
static BATCH_TIMEOUT: Duration = Duration::from_millis(200);
use dora_message::{message_capnp, serialize_message};

use dora_metrics::init_meter;
use dora_python_binding::PythonBinding;
//use dora_tracing::deserialize_context;
//use dora_tracing::{init_tracing, serialize_context};
use opentelemetry::global;
//use opentelemetry::{
//trace::{TraceContextExt, Tracer},
//Context,
//};
use opentelemetry_system_metrics::init_process_observer;

#[derive(Debug, PartialEq)]
struct Depth(u32);

#[tokio::main]
async fn main() -> eyre::Result<()> {
    let node = DoraNode::init_from_env().await?;

    // Opentelemetry Metrics
    let _started = init_meter();
    let meter = global::meter("process-meter");
    init_process_observer(meter);

    // Opentelemetry Tracing
    //  let tracer = init_tracing().unwrap();

    let py_function = PythonBinding::try_new(&node.id().to_string(), &"dora_run").unwrap();

    let is_source = node.node_config().inputs.len() == 0;

    let mut inputs = node.inputs().await?;

    loop {
        let mut workload = BTreeMap::new();
        let time_at_start = Instant::now();
        let py_function = py_function.clone();
        //   let mut max_depth = 0;
        //  let span = tracer.start(format!("{}.pushing", node.id()));
        //  let mut cx = Context::current_with_span(span).with_value(Depth(0));

        // Retrieve several inputs within a time frame
        if is_source {
            sleep(BATCH_TIMEOUT).await;
        } else {
            let input = match tokio::time::timeout(ATOMIC_TIMEOUT, inputs.next()).await {
                Ok(Some(input)) => input,
                Ok(None) => continue,
                Err(_) => continue,
            };

            //let deserialized = capnp::serialize::read_message(
            //&mut input.data.as_slice(),
            //capnp::message::ReaderOptions::new(),
            //)
            //.unwrap();
            //let message = deserialized
            //    .get_root::<message_capnp::message::Reader>()
            //    .unwrap();
            // let data = message.get_data().unwrap();
            //   let metadata = message.get_metadata().unwrap();
            //   let tmp_cx = deserialize_context(&metadata.get_otel_context().unwrap().to_string());
            //let depth = cx.get::<Depth>().unwrap_or(&Depth(0)).0;
            //if max_depth <= depth {
            //max_depth = depth;
            //cx = tmp_cx;
            //}
            workload.insert(input.id.to_string(), input.data.to_vec());
        }

        //   let span = tracer.start_with_context(format!("{}.pushing", node.id()), &cx);
        // cx = Context::current_with_span(span).with_value(Depth(max_depth + 1));
        //   let string_context = serialize_context(&cx);
        workload.insert("otel_context".to_string(), "".to_string().into_bytes());

        // Call the function
        let batch_messages = py_function.call(&workload).unwrap();

        // Send the data one by one.
        for (k, v) in batch_messages {
            node.send_output(&DataId::from(k), &v).await.unwrap();
        }
    }
}
