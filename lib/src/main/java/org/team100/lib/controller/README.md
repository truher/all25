# controller

The `lib.controller` package contains code that controls mechanisms, i.e. attempts
to drive a mechanism to a target position and/or velocity, by measuring the
actual mechanism state and applying some control law.

At the moment, the only control laws implemented are simple feedback,
proportional on position or full-state.  There's also a wrapper for
the WPI PID controller.

In the past, we had many more kinds of controllers, for example, pursuit followers,
LQR, min-time MPC, whatever.  Some of these ideas can be found in previous-year's
studies.  We found that complexity in control didn't really help anything,
and it was hard to understand, debug, and tune.