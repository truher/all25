# lib.sensor.position

This package supports position measurement using a variety of sensors.

There are two kinds of position measurement:

* [absolute](absolute/README.md)
* [incremental](incremental/README.md)

Like all sensors, position measurement classes are expected to produce
measurements that represent the current `Takt` instant, so they need to
consider sources of measurement delay.

Sensors wired to the RoboRIO have negligible delay.

An important distinction in measurement here is the notion of __wrapping__.  An
angular measurement can simply be a direction, like a clock: when the second
hand gets all the way around, it "rolls over" from 60 to 0.  Alternatively, the
measurement can be "wrapped" so that it doesn't roll over, as if the second hand
started counting 61, 62 ...  The "unwrapped" version is used for angular control
where we care about the total number of turns, e.g. a turret cannot spin
infinitely.