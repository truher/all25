# Conventions

Team 100 code uses a number of conventions in style and method.

## Style

Our naming conventions are similar to WPILib:

* Members start with "m_", so, ```double m_thing;```
* Constants are upper case, so ```private static final double I_AM_CONSTANT = 1.0;```
* Use the minimum appropriate visibility.  for most members, this is ```private.```
* Use ```final``` liberally.  most members should be ```final```.  Favor immutability in general: it saves you from mysteries at runtime.
* Package names are lower case, class names are title case.

The most important aspect of Team 100 code is simplicity.  Classes should be small.  Methods should be short.

Use names that describe the meaning of what you're doing.  If you change the meaning, change the name.  If the meaning is really obvious, it's ok to use really short names like ```x```.

When you have a lot of members and constructor arguments, put the lists in the same order.

Put variable initialization as close to use as possible.  The worst thing is to initialize all the variables at the top of a method, and then sprinkle the usage throughout the body.  Don't do that.

## Infrastructure

A common pattern is to print stuff to the console, if some sort of "debug" mode is engaged.  There are currently two patterns for that, a static boolean, or an interface that includes the printing methods -- the latter is preferred.

## Branching

If you know how to do it, it's encouraged to work on a branch in your fork, so that you can switch branches to work on more than one thing at a time.  If you don't know how to do it, don't worry about it.

## Units

We only ever use SI units: meters, radians, seconds, Newtons, and combinations thereof.  Never use inches or feet or degrees.

## Command composition

Keep commands simple and perpetual.  Expose "end" conditions that you can compose.  Don't embed timer-like features in your commands, just use WaitCommand.