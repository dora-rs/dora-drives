from typing import Callable

import cv2
import numpy as np
import torch
from dora import DoraStatus
from yolov7_tt100k import WEIGHTS, letterbox, non_max_suppression, scale_coords


class Operator:
    """
    Infering traffic sign from images
    """

    def __init__(self):
        model = torch.load(WEIGHTS)  # load
        self.model = (
            model["ema" if model.get("ema") else "model"].float().fuse().eval()
        )
        self.model.to(torch.device("cuda"))

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

        frame = cv2.imdecode(
            np.frombuffer(
                dora_input["data"],
                np.uint8,
            ),
            -1,
        )
        img = frame[:, :, :3]
        shape = img.shape
        img, ratio, (dw, dh) = letterbox(img)
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)
        img = torch.from_numpy(img).to(torch.device("cuda"))
        half = False
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        with torch.no_grad():
            pred = self.model(img)[0]
            pred = non_max_suppression(pred, 0.15, 0.15, None, False)[0]
            pred[:, :4] = scale_coords(
                img.shape[2:], pred[:, :4], shape
            ).round()
            results = pred.detach().cpu().numpy()

        arrays = np.array(results)[:, [0, 2, 1, 3, 4, 5]]  # xyxy -> xxyy
        arrays[:, 4] *= 100
        arrays = arrays.astype(np.int32)
        arrays = arrays.tobytes()
        send_output("bbox", arrays, dora_input["metadata"])
        return DoraStatus.CONTINUE
