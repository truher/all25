# lib.state

This package represents mechanisms:

- `Model100` represents measurements. Measurements never include acceleration,
since it is not directly measurable.
- `Control100` represents control outputs, which _do_ contain acceleration,
which can translate directly into motor voltages using the "kA" factor of the motor models.

In the "state space" representation in control theory, the `Model100` is
the `x` variable and `Control100` is the `u` variable.

There are groupings for the SE(2) manifold of 2d transformations, `ModelSE2`
and `ControlSE2`.  These treat each dimension independently, not using the
logmap geodesic constant-twist idea.