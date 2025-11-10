# lib.sensor.gyro

This package contains the `Gyro` interface and two implementations:

* `ReduxGyro` wraps our competition-standard gyro from Redux Robotics.
* `SimulatedGyro` uses inverse kinematics on the drivetrain wheels.