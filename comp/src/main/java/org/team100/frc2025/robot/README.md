# robot

The reason for the robot package is so that we can break up the large amount
of initialization we need to do into separate files, to reduce merge conflicts.

The classes here should be called at the end of the `Robot` constructor,
to do things like prewarming some code paths, binding buttons to commands,
and setting up autonomous commands.  These initializers generally need many
of the fields in `Robot`, so they're package-private.

This isn't a great design; it makes all of the Robot fields essentially
an API for configuration, but it seems better than making Robot huge,
or enumerating everything everywhere.

Everything should have the same pattern: you instantiate with the Robot
as the single constructor argument.