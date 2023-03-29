# Oasis Sensor


input of dora:

```python
sensors = [
            {
                "type": "sensor.camera.rgb",
                "id": "camera.center",
                "x": 2.0,
                "y": 0.0,
                "z": 1.5,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
                "width": IMAGE_WIDTH,
                "height": IMAGE_HEIGHT,
                "fov": 90,
            },
            {
                "type": "sensor.lidar.ray_cast",
                "id": "LIDAR",
                "x": 2.0,
                "y": 0.0,
                "z": 1.5,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            {
                "type": "sensor.other.gnss",
                "id": "GPS",
                "x": 2,
                "y": 0,
                "z": 1.5,
            },
            {
                "type": "sensor.other.imu",
                "id": "IMU",
                "x": 2,
                "y": 0,
                "z": 1.5,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            },
            {
                "type": "sensor.opendrive_map",
                "id": "高精地图传感器",
                "reading_frequency": 1,
            },
            {"type": "sensor.speedometer", "id": "速度传感器"},
```