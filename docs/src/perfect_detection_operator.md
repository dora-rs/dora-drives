# Perfect Detection Operator

The perfect object detection operator use information from the simulator to generate bounding box on image frame.

It uses the simulator to retrieve all positions of object within the simulation.

## Inputs

- segmented frame for information about what is visible to the car.
- depth frame to check if the object is not too far from the vehicle.
- position of the car to check the distances between obstacles.

## Outputs

- Bounding box with confidence and labels.