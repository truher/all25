package org.team100.lib.subsystems.r3;

import org.team100.lib.state.ModelR3;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SubsystemR3 extends Subsystem {
    /** State for the current Takt. */
    ModelR3 getState();

    /** Passthrough to motor stop. This is not "hold position", it is "disable". */
    void stop();

}
