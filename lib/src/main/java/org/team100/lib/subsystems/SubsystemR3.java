package org.team100.lib.subsystems;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A subsystem with three independent dimensions.
 */
public interface SubsystemR3 extends Subsystem {

    /** No scaling or filtering. */
    void setVelocity(GlobalVelocityR3 setpoint);

    /** State for the current Takt. */
    ModelR3 getState();

    /** Passthrough to motor stop. This is not "hold position", it is "disable". */
    void stop();


}