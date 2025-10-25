# motion

The `lib.motion` package supports actuation.

A Team 100 simple motion control consists of several components:

- a _motor_:  may support closed-loop positional or velocity control, or not.
- a _sensor_: usually the Team 100 magnetic absolute rotary sensor.
- a _mechanism_: combines the motor and sensor, and includes the gear ratio. 
- a _profile_: plans motion to limit velocity or acceleration
- _feedback_: tries to keep the mechanism on the profile
- a _servo_: a container for all of the above
- a _subsystem_: a configuration for all of the above, used by the scheduler
- some set of _commands_ that manipulates the subsystem

For examples of assembling these components into subsytems, and how to control them, look at lib/examples/motion.

One thing that's notable for its absence is any sort of high-level
cleverness about measurement, e.g. a Kalman filter, or anything
else that simulates mechanism dynamics.

We do expect __low-level__ cleverness about measurement: all measures
are expected to represent the current instant, which means that the
measuring classes should extrapolate, if possible, to account for
delay.