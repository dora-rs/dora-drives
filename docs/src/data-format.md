# Data format

All messages should be byte messages.

Best practice is to use C-order numpy arrays bytes.

they can be generated via the `.tobytes()` method from numpy arrays.

They can be read via `np.frombuffer`.

## Currently used message format

```python
## position of the car (1, 7) 
### qx, qy, qz, qw are angles quaternion.
position = np.array([x, y, z, qx, qy, qz, qw])

## frames (HEIGHT, WIDTH, 4)
frame = np.array([[[b, g, r, i], ... n_width ... ], ... n_height ... ])

## Obstacles without location (-1, 6)
bbox_2d = np.array([[min_x, max_x, min_y, max_y, confidence, label], ... n_bbox ... ])

## Obstacles with locations (-1, 5)
obstacles = np.array([[x, y, z, confidence, label], ... n_obstacle ... ])

## waypoints to follow. Shape (-1, 3)
waypoints = np.array([[x, y, speed], ... n_waypoint ... ])

## control for the car (1, 3)
control = np.array([throttle, steer, brake])
```