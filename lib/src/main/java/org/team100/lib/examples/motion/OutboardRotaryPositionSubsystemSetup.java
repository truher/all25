package org.team100.lib.examples.motion;

import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This is an example of what you'd put in Robot.java to use the subsystem.
 */
public class OutboardRotaryPositionSubsystemSetup {
    public OutboardRotaryPositionSubsystemSetup(LoggerFactory log, DriverXboxControl control) {

        /** Create an instance of the subsystem */
        OutboardRotaryPositionSubsystem rotary = new OutboardRotaryPositionSubsystem(log);

        /** Move to, and hold, the starting position, if nothing else is happening. */
        rotary.setDefaultCommand(rotary.home());

        /** Extend to a fixed location as long as the button is held. */
        new Trigger(control::x).whileTrue(rotary.extend());
    }

}
