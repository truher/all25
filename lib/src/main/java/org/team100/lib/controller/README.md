# controller

The `lib.controller` package contains code that controls mechanisms, i.e. attempts
to drive a mechanism to a target position and/or velocity, by measuring the
actual mechanism state and applying some control law.

There are two main interfaces:

* `drivetrain.SwerveController` was first used for the drivetrain, but is also appropriate for
any other control problem in the SE(2) Lie Group, i.e. planar translation and
rotation, which often occurs in planar mechanisms.  There's currently only one implementation,
which uses velocity feedforward, and proportional feedback on position and velocity.
In the past, we had many more kinds of controllers, for example, pursuit followers,
LQR, min-time MPC, whatever.  Some of these ideas can be found in previous-year's
studies.  We found that complexity in control didn't really help anything,
and it was hard to understand, debug, and tune.
* `simple.Feedback100` is for one-dimensional control problems.  There are implementations
with proportional position feedback, feedback on both position and velocity, and also
a wrapper for the WPI PID controller (on position).

