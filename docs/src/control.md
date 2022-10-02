# Control

To then translate those waypoints to an actual control, we're using a PID controller that is able to adjust the steering according to the response of the steering, and we're going to pipe this response to the CARLA API so that the car can move.

