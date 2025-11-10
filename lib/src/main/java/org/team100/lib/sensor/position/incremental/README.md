# lib.sensor.position.incremental

The key interface here is `IncrementalBareEncoder`.  It represents a sensor
that measures relative angular position but not absolute position.

For us, these are always implemented inside
the integrated motor controllers we use, though external versions do exist
elsewhere. This type of encoder is often good at measuring velocity.