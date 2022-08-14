import os
from typing import Callable

import cv2
import numpy as np
import tensorflow as tf

from dora_utils import DoraStatus


def load_coco_labels(labels_path):
    """Returns a map from index to label.
    Args:
        labels_path (:obj:`str`): Path to a file storing a label on each line.
    """
    labels_map = {}
    with open(labels_path) as labels_file:
        labels = labels_file.read().splitlines()
        index = 1
        for label in labels:
            labels_map[index] = label
            index += 1
    return labels_map


def load_serving_model(model_name, model_path, gpu_memory_fraction):
    detection_graph = tf.Graph()
    with detection_graph.as_default():
        # Load a frozen graph.
        graph_def = tf.compat.v1.GraphDef()
        with tf.io.gfile.GFile(model_path, "rb") as f:
            graph_def.ParseFromString(f.read())
            tf.import_graph_def(graph_def, name="")
    gpu_options = tf.compat.v1.GPUOptions(
        allow_growth=True,
        per_process_gpu_memory_fraction=gpu_memory_fraction,
    )
    return model_name, tf.compat.v1.Session(
        graph=detection_graph,
        config=tf.compat.v1.ConfigProto(gpu_options=gpu_options),
    )


MODEL_PATH = (
    os.environ["DORA_DEP_HOME"]
    + "/dependencies/models/obstacle_detection/efficientdet/efficientdet-d0/efficientdet-d0_frozen.pb"
)
# LABELS_PATH = os.environ["DORA_DEP_HOME"] + "/dependencies/models/coco.names"

IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600
OBSTACLE_THRESHOLD = 0.1
# coco_labels = load_coco_labels(LABELS_PATH)
models, tf_session = load_serving_model(
    "efficientdet-d0", model_path=MODEL_PATH, gpu_memory_fraction=0.9
)
signitures = {
    "image_files": "image_files:0",
    "image_arrays": "image_arrays:0",
    "prediction": "detections:0",
}


# Serve some junk image to load up the model.
inputs = np.zeros((IMAGE_HEIGHT, IMAGE_WIDTH, 3), dtype="uint8")
_ = tf_session.run(
    signitures["prediction"],
    feed_dict={signitures["image_arrays"]: [inputs]},
)[0]


class Operator:
    """
    Compute a `control` based on the position and the waypoints of the car.
    """

    def on_input(
        self,
        input_id: str,
        value: bytes,
        send_output: Callable[[str, bytes], None],
    ):

        camera_frame = cv2.imdecode(
            np.frombuffer(
                value[24:],
                dtype="uint8",
            ),
            -1,
        )
        resized_image = camera_frame[:, :, :3]
        input_image = np.ascontiguousarray(resized_image, dtype=np.uint8)
        outputs_np = tf_session.run(
            signitures["prediction"],
            feed_dict={signitures["image_arrays"]: [input_image]},
        )[0]

        obstacles = []

        for _, ymin, xmin, ymax, xmax, score, _class in outputs_np:
            xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)
            # if _class in coco_labels:
            if score >= OBSTACLE_THRESHOLD:
                xmin, xmax = max(0, xmin), min(xmax, IMAGE_WIDTH)
                ymin, ymax = max(0, ymin), min(ymax, IMAGE_HEIGHT)
                if xmin < xmax and ymin < ymax:
                    obstacles.append(
                        np.array(
                            [int(xmin), int(xmax), int(ymin), int(ymax)],
                            dtype=np.int32,
                        ).tobytes()
                    )

        send_output("obstacles", b"\n".join(obstacles))
        return DoraStatus.CONTINUE
