# hid

The `lib.hid` package is for human interface devices like joysticks, button boards, etc.

Team 100 does several unusual things with HIDs:

## Semantics

The control semantics are defined by interfaces:

* `DriverControl`
* `OperatorControl`
* `ThirdControl`

There are two main reasons for these interfaces:

* To allow multiple control bindings.  In particular, the controls used in simulation
have different implementations than the "real" controls.  We also have multiple options
for driver control: big joysticks, little joysticks, there's even an implementation
of absolute rotational control using a rotary sensor, and a button board implemented using
a MIDI keyboard (see `ThirdMidiControl`).
* To isolate the bindings themselves.  If there's no central place all the bindings
live, then they tend to be distributed, making it hard to answer questions like
"is button 3 available?" or "what does button 2 do?"

## Names

Team 100 HIDs are identified by name, so you don't have to synchronize anything in the
code with your controller choice: just plug it in and make sure it's mapped to the correct
[Driver Station USB slot](https://docs.wpilib.org/en/stable/docs/software/driverstation/driver-station.html#usb-devices-tab).
We use slot 0 for the `DriverControl`, slot 1 for the `OperatorControl`,
and slot 2 for the `ThirdControl`.

## Hot Swapping

Part of driver training can involve experimenting with multiple control types, and to
facilitate that, we use proxies (e.g. `DriverControlProxy`).  Each proxy periodically checks the
name of the attached device, and switches the implementation to match, if required.

These proxies do present a "trip hazard," which is that they need to fully implement the control
interface, or the interface defaults (which do nothing) will be used instead.

## Custom controls

See `all25/console` for some Arduino HID projects for custom controls, e.g. the MIDI controller,
or the many-many button controller used in 2025.