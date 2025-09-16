# encoder

The `lib.encoder` package supports position measurement using a variety of sensors.

There are various interface layers that make it easy to experiment with sensing.

Like all sensors, encoders are expected to produce measurements that represent the
current `Takt` instant, so they need to consider sources of measurement delay.

Sensors wired to the RoboRIO have negligible delay.