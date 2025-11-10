# lib.kinematics.urdf

We partially support Unified Robot Description Format (URDF), which is a
standard way to describe robot architectures that use "links" and "joints."
The most useful purpose for URDF is to numerically compute inverse kinematics,
which can be seen in `URDFRobot`.