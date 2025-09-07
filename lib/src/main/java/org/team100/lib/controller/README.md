# controller

The `lib.controller` package contains code that controls mechanisms, i.e. attempts
to drive a mechanism to a target position and/or velocity, by measuring the
actual mechanism state and applying some control law.

At the moment, the only control laws implemented are simple feedback,
proportional on position or full-state.  There's also a wrapper for
the WPI PID controller.