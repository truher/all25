package org.team100.lib.examples.motion;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ParallelRotarySetup {

    public ParallelRotarySetup(LoggerFactory log, DriverControl control) {
        RotaryPositionSubsystem1d r1 = new RotaryPositionSubsystem1d(log);
        RotaryPositionSubsystem1d r2 = new RotaryPositionSubsystem1d(log);
        new Trigger(control::button4).whileTrue(ParallelRotary.get(log, r1, r2));
    }
}
