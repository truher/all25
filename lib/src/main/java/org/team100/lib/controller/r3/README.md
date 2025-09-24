# r3

`lib.controller.r3` contains control methods for three dimensional
coordinates, where the three coordinates are orthogonal.  Our high-level
swerve control uses this sort of method, even though the SE(2) group
of Pose2d is not R3.  It is *tangent* to R3, and if the control
errors are small, R3 is an acceptable approximation to SE(2).

