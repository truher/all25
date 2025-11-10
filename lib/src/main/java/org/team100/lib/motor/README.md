# lib.motor

This package contains wrappers for various motors we might use.

The main interface is `BareMotor`, which describes a motor capable of control
via duty-cycle, velocity, or position: some implementations don't support all
of these control modes.

Team 100 motors never have intrinsic gearing; see `lib.motion.mechanism` for that.