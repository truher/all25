# lib.commands

This package contains mostly drivetrain commands that can
be assembled into more complex behaviors, most appropriately in the
`comp` folder (or wherever your code is), not in `lib` itself.

The Team100 style for commands encourages __perpetual__ commands,
i.e. `isFinished()` always returns false, so that the scheduler
never stops the command.  The reason is that it
is easier to compose commands in parallel that way: one command
can hold a "finished" position while a parallel command is still
running.  To facilitate this style, use the `MoveAndHold` interface.
