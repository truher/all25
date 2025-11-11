# lib.experiments

This package supports changes in behavior that can be
triggered at runtime.  Each experiment gets its own dashboard widget.
The expectation is that experiments will live for awhile, and then be
removed, as we learn what the "best" behavior is.  For example, you
could use an experiment to trigger conditional logic in a class, or change
the value of a parameter.

But we also use experiments as permanent switches, e.g. turning off 
the `HeedVision` experiment makes the robot ignore the camera input.

Some behavior changes require entirely different classes; for those, a good
pattern is to use an interface with two implementations, and some sort of
"selector" that chooses which one to execute.  Beware of maintaining state in
this situation.