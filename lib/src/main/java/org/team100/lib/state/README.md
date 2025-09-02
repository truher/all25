# State

The `lib.state` package represents mechanisms:

- `Model100` represents measurements. Measurements never include acceleration, since it is not directly measurable.
- `Control100` also represents control outputs, which _do_ contain acceleration, which can translate directly into motor voltages using the "kA" factor of the motor models.

In the "state space" representation in control theory, the `Model100` is the `x` variable.