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

## Obstacles
bounding_box_3d # np.array of shape
obstacle_transform = np.array([x, y, z, pitch, yaw, roll]).tobytes() # obstacle transform 
confidence # float 
actor_id # int
label # int
detailed_label # string 
obstacles = bounding_box_3d + obstacle_transform + actor_id + confidence + label + detailed_label

## waypoints to follow. Shape (-1, 3)
waypoints = np.array([x_array, y_array, speed_array]).tobytes()

## control for the car (1, 7)
control = np.array([throttle, steer, brake]).tobytes(),
```