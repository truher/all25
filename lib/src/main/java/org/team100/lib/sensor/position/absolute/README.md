# lib.sensor.position.absolute

The key interface here is `RotaryPositionSensor`.  It represents a sensor that
measures absolute angular position.

There are two main implementations, one is a real sensor attached to the RoboRIO,
which can work via the analog input or via dutycycle.  Our "go to" sensor is the
AMS AS5048 magnetic sensor, which uses the dutycycle interface.

There are also compositions that use a real sensor to "zero" an incremental
encoder, and then use the incremental encoder exclusively.  These are useful for
low-backlash mechanisms where the incremental encoder is used for outboard
closed loop positional control.

The real sensors are not good at velocity, though the compositions are.
It would be possible to use other kinds of sensors for this, e.g. the CTRE
CanCoder is one, but we don't use those.