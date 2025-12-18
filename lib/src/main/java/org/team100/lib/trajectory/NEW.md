# New Trajectories

## Background
Prior to 2026, Team 100 trajectories were implemented using code adapted from
254's 2022 repo.  This solution was holonomic (unlike the WPI version), and included
optimization of inner knots to minimize change in curvature.  The 254 code was written
in a very complex style, so the adaptation was a significant departure, but it still
contained many idiosyncrasies.

It also supported many things that just didn't seem to matter very much.  All our actual
trajectories for the past 2 seasons have had exactly one segment, so the optimizer
never even ran.  Even in tests it doesn't seem to do anything that can't be achieved through
careful tuning, which we do anyway.  And there was a class of things it couldn't do:
paths with corners, paths with only rotation, very short paths.  Worst of all were
the remaining idiosyncrasies of implementation: many copies of similar poses and
related data, mutability.  This was not something any student spent any time looking
closely at, it was just a black box.

For 2026, we should have a trajectory generation system that the students can understand,
end-to-end.  It should be much simpler, and without any of the 254 remnants.

## Design

As before, there will be several stages:

* Make a spline
* Sample the spline to make a piecewise linear path close enough to the real spline
* Assign timing, velocity, and acceleration to each point in the path

The representation will use SE(2), rather than the mix of R2 and SE(2) we used before.
This means that the distance metric will include rotation: for now we'll use the
L2 norm on all three components, with equal weighting.  We might come back to the
weighting if it seems to matter.

The SE(2) implementation will mean that rotation-only paths will work exactly the
same as linear or blended ones.

Each step in the process will be completely separate, unlike the previous version,
where the spline parameter (and derivatives) were used as part of the later steps.
Thus the scheduler should be able to work on *any* sequence of poses.

## Velocity

All three components of velocity of each sample are used for feedforward
and for constraints.  Because the acceleration within each segment is constant,
the velocities can be interpolated from the velocities at the endpoints,
or integrated from the starting end, given the acceleration at that point.

The previous scheduler computed (scalar, pathwise) velocity from the
(constrained) acceleration along a segment.  Heading velocity was
computed from the pathwise velocity using the direction of travel
("heading rate per meter").  This obviously won't work when the
pathwise velocity is zero, as it would be for turn-in-place segments.

For these computations, yet another annotation of the path is used,
with `ConstrainedState`, which remembers (pathwise, scalar, mutable)
velocity and acceleration.

Accordingly, the sampled datatype needs to have SE(2) velocity, whether
the integration or interpolation approach is used.

## Acceleration

All three components of acceleration of each sample are affected
by constraints, during scheduling.

It would also be nice to use acceleration for feedforward, like we do
for simpler mechanisms.

The acceleration within each segment is constant,
by definition, but it's not knowable by looking at the timing of the
endpoint poses alone. It *is* calculable given the velocities at the
endpoints.

The previous scheduler computed (scalar, pathwise) acceleration,
and records it in `TimedPose.`

Both pathwise and centripetal acceleration are available in the Control
copy of the TimedPose, where it seems to be used in the Jacobian
(for planar mechanisms) -- it woudl be good to use this for driving too.

Accordingly, the sampled datatype needs to have the SE(2) acceleration too.

## Path Generation

We've gone back and forth on pathing on the manifold or pathing on the tangent space,
with just the planar Euclidean metric.  Using the manifold, for samples far apart,
makes noticeable "scalloping" of the resulting path.  Of course, using the tangent
space also produces "facets," but they're straight.

For this design, paths will be in the manifold, which means scallops, so the
samples should be relatively close together.

## Scheduling

Given a sequence of poses, the scheduler connects them with constant-direction arcs
(twists) in SE(2) that are traversed with constant acceleration.

Using constant-direction arcs means that all the information about the path is given
by the poses themselves: there's no reason to use the spline to produce anything
about heading rate or curvature.
