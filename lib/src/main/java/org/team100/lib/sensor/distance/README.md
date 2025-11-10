# lib.sensor.distance

A distance sensor is something like an ultrasonic or time-of-flight rangefinder,
measuring the distance to an object without touching it.

At the moment we have just one class here: `LaserCan100` which wraps a real,
or mock, LaserCAN device.