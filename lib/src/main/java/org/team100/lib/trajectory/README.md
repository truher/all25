# lib.trajectory

This package represents an path in SE(2) and a schedule.

`Trajectory100` is adapted from 254.  Trajectories are holonomic,
i.e. handle course and heading separately, so they have four dimensions (x, y, heading, course), and "point" below means four-dimensional holonomic state.

A good entry point to experiment with trajectories is `TrajectoryPlanner`.  Try `restToRest()` between two poses.

The process of constructing a trajectory has three stages:

1. Construct a list of splines.  Each spline joins two points smoothly.  The spline parameter has no physical meaning, it's just 0.0 on one end and 1.0 on the other.  See the `lib.trajectory.path.spline` package.

2. Construct a list of points along the spline, such that straight lines connecting the points ("secant lines") don't deviate too much from the true spline. (This uses recursive bisection.)  These points will be close together where the curvature is high, and far apart along straighter sections of the spline.  The list is created by `PathFactory`, producing `Path100`, which integrates along the list to find the distance.  See the `lib.trajectory.path` package.

3. Construct a list of points interpolated along the secant lines, such that the points aren't too far apart.

4. Using a list of kinodynamic constraints (see `lib.trajectory.timing`), assign a time for each point.  The resulting list of `TimedPose`s is created by `ScheduleGenerator`, producing `Trajectory100`.

To use a trajectory, you `sample()` it, with time (in seconds) as the parameter.  The resulting `TimedPose` is interpolated between from the list above.

If you want to use these trajectories for non-holonomic (e.g. "tank") drivetrains, it will work well enough to set the course and heading to be the same at each waypoint.

## Math

The process above is confusing.  Here's what it should do instead.

1. make a spline based on the endpoints
2. construct a list of *spline parameter values* that satisfy the secant constraint
3. construct a discrete mapping from those spline parameters to time
4. at runtime, interpolate

there could be some caching of the spline values to save some multiplications.

the key is to not keep so many copies of the pose data around; there's no reason for it
since we don't do optimization anymore.

The essential math here is as follows.

The spline position is $q(s)$ for the parameter $s$

The velocity with respect to $s$ is $q'(s) = dq/ds$

The accleration with respect to $s$ is $q''(s) = d^2q/ds^2$

So we're constructing a function for $s(t)$ and its time derivatives,

$\dot{s} = \dfrac{ds}{dt}$

$\ddot{s} = \dfrac{d^2s}{dt^2}$

Position is just the composite:
$x(t) = q(t) = q(s(t))$

so using the chain rule:
$v(t) = \dot{q}(t) = \dfrac{dq}{ds} \dfrac{ds}{dt} = q'(s(t))\dot{s}$

and using the product rule:
$a(t) = \ddot{q}(t) = q''(s)\dot{s}^2 + q'(s)\ddot{s}$

## Constructing $s(t)$

The way it works now, the last step in `timeParameterizeTrajectory` is to integrate through the
constrained states.

* Each state has a "total distance so far" number; this is used to find the length of the previous segment.
* each state has a velocity number (real velocity wrt time).
* the acceleration during the previous segment is calculated based on the difference in velocity and the length.  this acceleration is attached to the *previous* state, i.e. the state velocity is instantaneous but the acceleration applies to the entire following segment.
* The duration of the previous segment is calculated based on the difference in velocities and the derived acceleration above
* segment duration is cumulated
* each state is annotated with the cumulative time, the endpoint velocity, and zero for the acceleration (it is filled in on the next loop).

## Sampling

at runtime, the `TimedPose` list is sampled by time, which means finding floor and ceiling
states and interpolating.  The interpolant is the fraction of time between states, which
is used to derive the fraction of distance between states (i.e. along the secant line),
which is used as the interpolant between poses.  Note the acceleration is not interpolated
since it is constant within a segment.

Is caching states and interpolating faster than computing the spline on the fly?

No!  see `Trajectory100Test.testSamplePerformance` which shows that it's about the same, 
around 200 ns per sample.  The budget is 20000000 ns, so the sampling time is 
one millionth of the budget.