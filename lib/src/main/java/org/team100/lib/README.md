# lib

The `lib` package contains library code we can use every year.

Everything in here needs to be reviewed and covered with tests.

If you're trying to learn about the library, you should start with the README in each package.

Of particular note:

* [coherence](coherence/README.md)
  * A discrete clock and cache intended to manage observations (so they all represent the same instant) and anything else that depends on time (e.g. steps along a profile).
* [commands](commands/README.md)
  * High level navigation commands, using manual control, profiles, and trajectories.
* [examples](examples/README.md) 
  * A few illustrations of using library features.
* [experiments](experiments/README.md) 
  * switches that control optional features using dashboard selectors.
* [hid](hid/README.md) 
  * We push the "control domain" (what each button *means*) into the controller API, and allow hot-swapping controllers.
* [logging](logging/README.md) 
  * Loggers are passed through constructors, so the tree in Network Tables mirrors the construction graph, as a starting point.
* [motion](motion/README.md) 
  * High-level classes for motion control and kinematics
* [motor](motor/README.md) 
  * Wrappers for all the motors we use (including simulated motors), so they can be handled uniformly by the motion classes above.
* [profile](profile/README.md)
  * Constrained motion in 1d and 2d-with-heading.  These should be used for all motion where a trajectory would be too expensive to compute on the fly.
* [trajectory](trajectory/README.md)
  * A trajectory is a path based on splines, with a precalculated schedule meeting motion constraints.  Trajectories are good for paths that require curves around known obstacles.  Simple trajectories are not *that* time-consuming to create, so can be used on-the-fly in some cases.