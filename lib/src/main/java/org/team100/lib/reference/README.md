# reference

The `lib.reference` package supplies "references" (i.e. controller setpoints)
with various dimensionalities and sources, e.g. pre-planned trajectories or
on-the-fly constrained profiles.  This decouples the controller code from
the reference source.

An important trait of Team 100 reference generators is that they produce two
setpoints: one for the "current time step" and one for the "next time step."
The reason to track these separately is that they are used for different
purposes:

* The "current" setpoint represents where the mechanism should be at the
current instant in time (defined by `Takt`), and so it should be compared
with the current measurement (as obtained by `Cache`), to derive the
the controller error, and form the basis of __feedback control.__  For example,
the simplest feedback control law would be to simply feed the (scaled) error
into the output: this is called __proportional feedback__.
* The "next" setpoint represents where the mechanism should be at the next
`Takt` instant.  This is useful to guide __feedforward control__.  Since
most of our actuators are velocity servos, most of our feedforwards are
simply "next setpoint velocity" passthroughs.