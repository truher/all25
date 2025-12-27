# Subsystems

These general interfaces make it easier to reuse controllers etc
for different, but similarly controlled, subsystems.

For example, a Mecanum drivetrain and a Swerve drivetrain have some things
in common, and are both velocity controlled in SE(2), so they are
implementations of `VelocitySubsystemSE2`.