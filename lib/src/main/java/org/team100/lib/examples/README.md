# lib.examples

This package contains common subsystem and
command setups can use in a real robot.

The "setup" can just be added to the RobotContainer constructor.
It creates some commands and binds them to the control, and this
binding keeps them from being garbage collected.

Each "setup" can persist in the RobotContainer, so you can propagate
periodics to it (e.g. for visualization).