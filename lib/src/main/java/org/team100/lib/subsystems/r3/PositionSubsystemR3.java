package org.team100.lib.subsystems.r3;

import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * A subsystem with three independent dimensions, controlled by position.
 * 
 * We use this interface for mechanisms whose position can be controlled via
 * outboard closed-loop controllers.
 */
public interface PositionSubsystemR3 extends Subsystem {

    /** Position, velocity, and acceleration may all be used. */
    void set(ControlR3 setpoint);

    /** State for the current Takt. */
    ModelR3 getState();

    /** Passthrough to motor stop. This is not "hold position", it is "disable". */
    void stop();

}