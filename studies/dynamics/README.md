# Dynamics

This is a study of mechanism dynamics for use on the Calgames robot.

The Calgames robot uses a three-jointed mechanism, "Prismatic-Revolute-Revolute"
or PRR, which is controlled via position feedback (via inverse kinematics),
velocity feed-forward (using the joint Jacobian), and gravity.  The missing
piece is torque feedforward, which can account for the influence of one
joint on the others.  Imagine the elevator going up very fast, with the arm
held out to the side: without compensation, what will happen to the arm?
It will feel a "downward" force due to its inertia, and this force will
result in positional controller error.  We had trouble with this coupling
last season, with the wrist flailing around due to the motion of the
elevator.

To start with, let's work out the dynamics of a simpler, two-joint mechanism,
[Prismatic-Revolute {PR}](doc/README_PR.md)

