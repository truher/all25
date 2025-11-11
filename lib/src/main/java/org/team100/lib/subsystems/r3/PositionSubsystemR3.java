package org.team100.lib.subsystems.r3;

import org.team100.lib.state.ControlR3;

/**
 * A subsystem with three independent dimensions, controlled by position.
 * 
 * We use this interface for mechanisms whose position can be controlled via
 * outboard closed-loop controllers.
 */
public interface PositionSubsystemR3 extends SubsystemR3 {

    /** Position, velocity, and acceleration may all be used. */
    void set(ControlR3 setpoint);
}