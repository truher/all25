# lib.profile

This package supports "profiled" motion.

There are two types, "timed" and "incremental."

A `TimedProfile` is like a trajectory: you precalculate the schedule
and then sample it.

An `IncrementalProfile` has no state: you give it the current setpoint,
and it produces the next one.  There are also a few methods related to
coordinating multiple profiles so that they arrive at their goals
at the same time, e.g. for controlling multiple-DOF mechanisms,
or the drivetrain.

There is also `HolonomicProfile` which simply wraps (any) set of
three `IncrementalProfile`s and coordinates them.