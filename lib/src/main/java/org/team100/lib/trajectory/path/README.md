# lib.trajectory.path

This package includes `Path100`, which describes a 2d holonomic path
with a list of samples of a spline.

The `PathFactory` samples a spline so that the straight parts don't have too
many points, but the curved parts have more.