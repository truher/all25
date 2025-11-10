# lib.optimization

This package contains iterative solvers used in applications
like inverse kinematics and profile coordination, notably `NewtonsMethod`
for finding the zero of a function.  Since these solvers are iterative,
you'll want to be careful with how long they take.

There's also `NumericalJacobian100`, which finds the
[Jacobian](https://en.wikipedia.org/wiki/Jacobian_matrix) of any
function using finite differences.  It's useful for, e.g. mechanism
velocity kinematics.  It is surprisingly fast, even on the RoboRIO,
for the low-dimensionality functions we usually use, and it's certainly
easier than writing the analytic Jacobian by hand.
