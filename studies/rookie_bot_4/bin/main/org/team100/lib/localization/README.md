# localization

The `lib.localization` package uses two sources of position data
to determine the robot's global pose:

## Odometry

At each time step, the difference in wheel positions since the previous
time step is used to compute a difference in pose, using forward kinematics.
Obviously odometry can only estimate pose differences, not absolute pose.
See `OdometryUpdater`.

## Vision

Each camera publishes relative poses of AprilTags it can see (`Blip24`s), and
these are received by `AprilTagRobotLocalizer`, which transforms each one into
a global pose estimate, using its knowledge of each camera pose relative to the
robot (see `Camera`), and each tag relative to the field
(see `AprilTagFieldLayoutWithCorrectOrientation`).

The main complexity here is __accommodating delay.__  Camera input is delayed between
around 75 and 100, and so it can't be used to directly adjust the robot's __current__ pose
estimate.  Instead, we apply camera input to __old__ pose estimates, recorded by
`SwerveHistory`, and then __replay__ the subsequent odometry differentials to get
the current pose.

The camera input doesn't completely override the old estimate, we use a weighted
average (see `NudgingVisionUpdater`).  The weights can be adjusted for every update,
typically we weigh the vision less when the tag is far away.

Another important subtlety is that when we transform camera-relative tag sightings
into robot pose estimates, we don't use the rotational component of the tag sighting,
we use the `Gyro` reading instead, because the gyro is more accurate.  But this does
mean that the vision system is dependent on the gyro: if the gyro zero is wrong,
then the vision will be confused.  Also the gyro does drift (about 2 millirad/s)
so you'll have to reset the robot rotation periodically, and the gyro, like every
sensor, includes noise(about 0.05 rad for
[the gyro we use](https://docs.reduxrobotics.com/canandgyro/performance)).
Thus, far-away tag sightings combined
with rotational noise can produce translational error, around 5 percent
of the tag distance.  So for a tag 2 meters away, a translational estimate could
be off by 10 cm due to gyro noise alone, which is a large error!  This is one
of the reasons we use a weighted average for vision updates: it acts as a
low-pass filter.

