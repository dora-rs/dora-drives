## Data Format

All messages should be byte messages.

Best practice is to use C-order numpy arrays bytes.

they can be generated via the `.tobytes()` method.

They can be read via `np.frombuffer`.

## Currently used message format

```python
## position of the car (1, 7)
position = np.array([x, y, z, pitch, yaw, roll, forward_speed]).tobytes()

## frames 
raw_data # image in bytes of uint8 with shape (width, height, channel)
camera_transform = np.array([x, y, z, pitch, yaw, roll]).tobytes() # Camera settings 
frame = raw_data + camera_transform

## Obstacles without location
bounding_box_2d = np.array([min_x, max_x, min_y, max_y])
obstacle_camera_transform = np.array([x, y, z, pitch, yaw, roll]).tobytes() # obstacle transform 
confidence # float 
label # int
obstacles = bounding_box_2d + obstacle_camera_transform + confidence + label 

## Obstacles with locations
obstacle_transform = np.array([[x, y, z, pitch, yaw, roll], ...]).tobytes() # obstacle transform 
confidence # float 
label # int
obstacles = bounding_box_3d + obstacle_transform + confidence + label 

## waypoints to follow. Shape (-1, 3)
waypoints = np.array([x_array, y_array, speed_array]).tobytes()

## control for the car (1, 7)
control = np.array([throttle, steer, brake]).tobytes(),
```