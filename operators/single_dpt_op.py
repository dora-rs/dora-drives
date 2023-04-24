import os
from typing import Callable

import cv2
import numpy as np
import torch
# import matplotlib.pyplot as plt
from dora import DoraStatus

IMAGE_WIDTH = 1920
IMAGE_HEIGHT = 1080
DEVICE = os.environ.get("PYTORCH_DEVICE") or "cpu"
DPT_PATH = os.environ.get("DPT_PATH")
DPT_WEIGHT_PATH = os.environ.get("DPT_WEIGHT_PATH")
MODEL_TYPE = os.environ.get("MODEL_TYPE")

# example source: https://pytorch.org/hub/intelisl_midas_v2/
class Operator:
    def __init__(self):
        if DPT_PATH is None:
            # With internet
            self.model = torch.hub.load(
                "intel-isl/MiDaS",
                MODEL_TYPE,
            )
        else:
            # Without internet
            self.model = torch.hub.load(
                DPT_PATH,
                "custom",
                path=DPT_WEIGHT_PATH,
                source="local",
            )
        midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
        if MODEL_TYPE == "DPT_Large" or MODEL_TYPE == "DPT_Hybrid":
            self.transform = midas_transforms.dpt_transform
        else:
            self.transform = midas_transforms.small_transform
        self.model.to(torch.device(DEVICE))
        self.model.eval()

    def on_event(
            self,
            dora_event: dict,
            send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        if dora_event["type"] == "INPUT":
            return self.on_input(dora_event, send_output)
        return DoraStatus.CONTINUE

    def on_input(
            self,
            dora_input: dict,
            send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:
        """Handle image
        Args:
            dora_input["id"](str): Id of the input declared in the yaml configuration
            dora_input["data"] (bytes): Bytes message of the input
            send_output (Callable[[str, bytes]]): Function enabling sending output back to dora.
        """
        if dora_input["id"] == "image":
            frame = np.frombuffer(
                dora_input["data"],
                np.uint8,
            ).reshape((IMAGE_HEIGHT, IMAGE_WIDTH, 4))
            
            with torch.no_grad():
                image = frame[:, :, :3]
                print("frame: ", frame.shape)
                img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                print("img: ", img.shape)
                input_batch = self.transform(img).to(DEVICE)
                prediction = self.model(input_batch)
                prediction = torch.nn.functional.interpolate(
                    prediction.unsqueeze(1),
                    size=img.shape[:2],
                    mode="bicubic",
                    align_corners=False,
                ).squeeze()
                
                output = prediction.cpu().numpy()
                print("output: ", output)
                # plt.imshow(output)
                # plt.show()
                send_output("depth_frame", output.tobytes(), dora_input["metadata"])
            print("frame exist")
        return DoraStatus.CONTINUE