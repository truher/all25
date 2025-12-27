# lib.logging

This package is what we use to put stuff on the dashboard
and in the log file.

The notable features of Team 100 logging are

* Levels, to ignore some log events in the interest of performance.  logging is slow. 
  * COMP: for things we always want to log, even at competition
  * DEBUG: things we look at a lot
  * TRACE: absolutely everything
* Alternate transport.  if we really want to log a whole lot, we have an alternate primitive transport available, using an outboard (Raspberry Pi) log receiver.  It would be good to not use this, but it's available if we really need it.

The general pattern is:

* create a `LoggerFactory` the root level
* add levels by name or type
* pass the `LoggerFactory` through the tree of constructors
* in each constructor, make a type- or name- specific `LoggerFactory`
* use it to get type-specific logger instances, e.g. `DoubleLogger`
* elsewhere, call `log` with a supplier

So the resulting tree in Network Tables looks like the instantiation graph.

The reason the loggers take suppliers is so that expensive operations can be skipped
if their results aren't going to be logged under the current Level anyway.