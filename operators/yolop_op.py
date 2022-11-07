from typing import Callable
from enum import Enum

import cv2
import numpy as np
import torch
import torchvision
import torchvision.transforms as transforms


class DoraStatus(Enum):
    CONTINUE = 0
    STOP = 1


normalize = transforms.Normalize(
    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
)

transform = transforms.Compose([transforms.ToTensor(), normalize])


def xywh2xyxy(x):
    # Convert nx4 boxes from [x, y, w, h] to [x1, y1, x2, y2] where xy1=top-left, xy2=bottom-right
    y = torch.zeros_like(x) if isinstance(x, torch.Tensor) else np.zeros_like(x)
    y[:, 0] = x[:, 0] - x[:, 2] / 2  # top left x
    y[:, 1] = x[:, 1] - x[:, 3] / 2  # top left y
    y[:, 2] = x[:, 0] + x[:, 2] / 2  # bottom right x
    y[:, 3] = x[:, 1] + x[:, 3] / 2  # bottom right y
    return y


def non_max_suppression(
    prediction,
    conf_thres=0.65,
    iou_thres=0.25,
):
    """Performs Non-Maximum Suppression (NMS) on inference results
    Returns:
         detections with shape: nx6 (x1, y1, x2, y2, conf, cls)
    """

    output = [torch.zeros((0, 6), device=prediction.device)] * prediction.shape[
        0
    ]

    for xi, x in enumerate(prediction):  # image index, image inference
        # Apply constraints
        # x[((x[..., 2:4] < min_wh) | (x[..., 2:4] > max_wh)).any(1), 4] = 0  # width-height

        # Compute conf
        confidence = x[:, 5:] * x[:, 4:5]  # conf = obj_conf * cls_conf

        # Box (center x, center y, width, height) to (x1, y1, x2, y2)
        box = xywh2xyxy(x[:, :4])

        # Detections matrix nx6 (xyxy, conf, cls)
        conf, j = confidence.max(1, keepdim=True)
        x = torch.cat((box, conf, j.float()), dim=1)[conf.view(-1) > conf_thres]

        # Check shape
        if not x.shape[0]:  # no boxes
            continue
        # Batched NMS
        boxes, scores = x[:, :4], x[:, 4]  # boxes (offset by class), scores
        i = torchvision.ops.nms(boxes, scores, iou_thres)  # NMS

        output[xi] = x[i]
    return output


def morphological_process(image, kernel_size=5, func_type=cv2.MORPH_CLOSE):
    """
    morphological process to fill the hole in the binary segmentation result
    :param image:
    :param kernel_size:
    :return:
    """
    if len(image.shape) == 3:
        raise ValueError(
            "Binary segmentation result image should be a single channel image"
        )

    if image.dtype is not np.uint8:
        image = np.array(image, np.uint8)

    kernel = cv2.getStructuringElement(
        shape=cv2.MORPH_ELLIPSE, ksize=(kernel_size, kernel_size)
    )

    # close operation fille hole
    closing = cv2.morphologyEx(image, func_type, kernel, iterations=1)

    return closing


def if_y(samples_x):
    for sample_x in samples_x:
        if len(sample_x):
            # if len(sample_x) != (sample_x[-1] - sample_x[0] + 1) or sample_x[-1] == sample_x[0]:
            if sample_x[-1] == sample_x[0]:
                return False
    return True


def fitlane(mask, sel_labels, labels, stats):
    H, W = mask.shape
    lanes = []
    for label_group in sel_labels:
        states = [stats[k] for k in label_group]
        x, y, w, h, _ = states[0]
        # if len(label_group) > 1:
        #     print('in')
        #     for m in range(len(label_group)-1):
        #         labels[labels == label_group[m+1]] = label_group[0]
        t = label_group[0]
        # samples_y = np.linspace(y, H-1, 30)
        # else:
        samples_y = np.linspace(y, y + h - 1, 30)

        samples_x = [
            np.where(labels[int(sample_y)] == t)[0] for sample_y in samples_y
        ]

        if if_y(samples_x):
            samples_x = [
                int(np.mean(sample_x)) if len(sample_x) else -1
                for sample_x in samples_x
            ]
            samples_x = np.array(samples_x)
            samples_y = np.array(samples_y)
            samples_y = samples_y[samples_x != -1]
            samples_x = samples_x[samples_x != -1]
            func = np.polyfit(samples_y, samples_x, 2)
            x_limits = np.polyval(func, H - 1)
            # if (y_max + h - 1) >= 720:

            draw_y = np.linspace(y, y + h - 1, 30)
            draw_x = np.polyval(func, draw_y)
            # draw_y = draw_y[draw_x < W]
            # draw_x = draw_x[draw_x < W]
            lanes.append((np.asarray([draw_x, draw_y]).T).astype(np.int32))
        else:
            # if ( + w - 1) >= 1280:
            samples_x = np.linspace(x, W - 1, 30)
            # else:
            #     samples_x = np.linspace(x, x_max+w-1, 30)
            samples_y = [
                np.where(labels[:, int(sample_x)] == t)[0]
                for sample_x in samples_x
            ]
            samples_y = [
                int(np.mean(sample_y)) if len(sample_y) else -1
                for sample_y in samples_y
            ]
            samples_x = np.array(samples_x)
            samples_y = np.array(samples_y)
            samples_x = samples_x[samples_y != -1]
            samples_y = samples_y[samples_y != -1]
            try:
                func = np.polyfit(samples_x, samples_y, 2)
            except:
                print("polyfit did not work")
            # y_limits = np.polyval(func, 0)
            # if y_limits > 720 or y_limits < 0:
            # if (x + w - 1) >= 1280:
            #     draw_x = np.linspace(x, 1280-1, 1280-x)
            # else:
            draw_x = np.linspace(x, x + w - 1, 30)
            draw_y = np.polyval(func, draw_x)
            lanes.append((np.asarray([draw_x, draw_y]).T).astype(np.int32))
            # cv2.polylines(mask, [draw_points], False, 1, thickness=3)
    return lanes


def connect_lane(
    image, shadow_height=0, kernel_size=7, func_type=cv2.MORPH_OPEN
):
    if len(image.shape) == 3:
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray_image = image
    if shadow_height:
        image[:shadow_height] = 0
    mask = np.zeros((image.shape[0], image.shape[1]), np.uint8)

    num_labels, labels, stats, centers = cv2.connectedComponentsWithStats(
        gray_image, connectivity=8, ltype=cv2.CV_32S
    )
    # ratios = []
    selected_label = []

    for t in range(1, num_labels, 1):
        _, _, _, _, area = stats[t]
        if area > 400:
            selected_label.append(t)
    if len(selected_label) == 0:
        return mask
    else:
        split_labels = [
            [
                label,
            ]
            for label in selected_label
        ]
        points = fitlane(mask, split_labels, labels, stats)
        return points

    # close operation fill hole
    closing = cv2.morphologyEx(image, func_type, kernel_size, iterations=1)

    return closing


def letterbox_for_img(
    img,
    new_shape=(640, 640),
    color=(114, 114, 114),
    auto=True,
    scaleFill=False,
    scaleup=True,
):
    # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))

    dw, dh = (
        new_shape[1] - new_unpad[0],
        new_shape[0] - new_unpad[1],
    )  # wh padding

    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, 32), np.mod(dh, 32)  # wh padding

    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = (
            new_shape[1] / shape[1],
            new_shape[0] / shape[0],
        )  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2
    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_AREA)

    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(
        img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )  # add border
    return img, ratio, (dw, dh)


class Operator:
    """
    Infering object from images
    """

    def __init__(self):
        self.model = torch.hub.load("hustvl/yolop", "yolop", pretrained=True)
        self.model.to(torch.device("cuda"))
        self.model.eval()

    def on_input(
        self,
        dora_input: dict,
        send_output: Callable[[str, bytes], None],
    ) -> DoraStatus:

        # inference
        frame = cv2.imdecode(
            np.frombuffer(
                dora_input["data"],
                dtype="uint8",
            ),
            -1,
        )

        frame = frame[:, :, :3]
        h0, w0, _ = frame.shape
        h, w = (640, 640)
        frame, _, (pad_w, pad_h) = letterbox_for_img(frame)
        ratio = w / w0
        pad_h, pad_w = (int(pad_h), int(pad_w))

        img = torch.unsqueeze(transform(frame), dim=0)
        half = False  # half precision only supported on CUDA
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img = img.to(torch.device("cuda"))
        det_out, da_seg_out, ll_seg_out = self.model(img)

        # det_out = [pred.reshape((1, -1, 6)) for pred in det_out]
        # inf_out = torch.cat(det_out, dim=1)

        # det_pred = non_max_suppression(
        # inf_out,
        # )
        # det = det_pred[0]

        da_predict = da_seg_out[
            :, :, pad_h : (h0 - pad_h), pad_w : (w0 - pad_w)
        ]
        da_seg_mask = torch.nn.functional.interpolate(
            da_predict, scale_factor=1 / ratio, mode="bilinear"
        )
        _, da_seg_mask = torch.max(da_seg_mask, 1)
        da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
        da_seg_mask = morphological_process(da_seg_mask, kernel_size=7)

        contours, _ = cv2.findContours(
            da_seg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if len(contours) != 0:
            contour = max(contours, key=cv2.contourArea)
            contour = contour.astype(np.int32)
            send_output(
                "drivable_area", contour.tobytes(), dora_input["metadata"]
            )
        else:
            send_output(
                "drivable_area", np.array([]).tobytes(), dora_input["metadata"]
            )

        ll_predict = ll_seg_out[
            :, :, pad_h : (h0 - pad_h), pad_w : (w0 - pad_w)
        ]

        ll_seg_mask = torch.nn.functional.interpolate(
            ll_predict, scale_factor=1 / ratio, mode="bilinear"
        )

        _, ll_seg_mask = torch.max(ll_seg_mask, 1)
        ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()
        # Lane line post-processing
        ll_seg_mask = morphological_process(
            ll_seg_mask, kernel_size=7, func_type=cv2.MORPH_OPEN
        )
        ll_seg_points = np.array(connect_lane(ll_seg_mask), dtype=np.int32)
        send_output("lanes", ll_seg_points.tobytes(), dora_input["metadata"])
        return DoraStatus.CONTINUE
