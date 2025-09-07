package org.team100.lib.examples.motion;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OpenLoopSubsystemSetup {
    public OpenLoopSubsystemSetup(LoggerFactory log, DriverControl control) {
        OpenLoopSubsystem openloop = new OpenLoopSubsystem(log);

        /**
         * Binds some buttons to the open loop system.
         */
        new Trigger(control::button4).whileTrue(openloop.forward());
        new Trigger(control::button5).whileTrue(openloop.reverse());
    }
}
