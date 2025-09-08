# state

`lib.motion.drivetrain.state` contains simple containers for stuff about
the drivetrain, e.g. the position of each wheel, the velocity of the
drivetrain in field-relative terms, etc.

The drivetrain state has two representations:

- `SwerveModel` represents measurements. Measurements never include
acceleration, since it is not directly measurable.  Measurements are 
expected to represent the current instant, if they're derived from
some sort of sensor, so sensor implementations should extrapolate
to correct delay if required.
- `SwerveControl` represents control outputs, which _do_ contain
acceleration, which can translate directly into motor voltages using
the "kA" factor of the motor models.
