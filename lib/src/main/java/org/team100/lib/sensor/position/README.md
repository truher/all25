# encoder

The `lib.encoder` package supports position measurement using a variety of sensors.

There are various interface layers that make it easy to experiment with sensing.

Like all sensors, encoders are expected to produce measurements that represent the
current `Takt` instant, so they need to consider sources of measurement delay.

Sensors wired to the RoboRIO have negligible delay.

The key interfaces here are:

* `IncrementalBareEncoder`: represents a sensor that measures relative angular position
but not absolute position.  For us, these are always implemented inside
the integrated motor controllers we use, though external versions do exist
elsewhere. This type of encoder is often good at measuring velocity.
* `RotaryPositionSensor`: represents a sensor that measures absolute angular position.
There are two main implementations, one is a real sensor attached to the RoboRIO,
and there are also compositions that use a real sensor to "zero" an incremental
encoder.  The real sensors are not good at velocity, though the compositions are.
It would be possible to use other kinds of sensors for this, e.g. the CTRE
CanCoder is one, but we don't use those.

An important distinction in measurement here is the notion of __wrapping__.  An
angular measurement can simply be a direction, like a clock: when the second
hand gets all the way around, it "rolls over" from 60 to 0.  Alternatively, the
measurement can be "wrapped" so that it doesn't roll over, as if the second hand
started counting 61, 62 ...  The "unwrapped" version is used for angular control
where we care about the total number of turns, e.g. a turret cannot spin
infinitely.