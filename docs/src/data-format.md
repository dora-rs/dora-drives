# Data format

All messages should be byte messages.

Best practice is to use C-order numpy arrays bytes.

they can be generated via the `.tobytes()` method from numpy arrays.

They can be read via `np.frombuffer`.

## Currently used message format

```python
## position of the car (1, 7)
position = np.array([x, y, z, pitch, yaw, roll, forward_speed])

## frames 
frame = raw_data # image in bytes of uint8 encoded in jpeg.

## Obstacles without location (-1, 6)
bounding_box_2d = np.array([[min_x, max_x, min_y, max_y, confidence, label], ...])

## Obstacles with locations (-1, 5)
obstacles = np.array([[x, y, z, confidence, label], ...])

## waypoints to follow. Shape (-1, 3)
waypoints = np.array([x_array, y_array, speed_array])

## control for the car (1, 3)
control = np.array([throttle, steer, brake])
```