# Yolov5 operator

`Yolov5` object detection operator generates bounding boxes on images where it detects object. 

`Yolov5` has not been finetuned on the simulation and is directly importing weight from Pytorch Hub.

## Inputs

- jpeg encoded image as input.

## Outputs

- Bounding box coordinates as well as the confidence and class label as output.