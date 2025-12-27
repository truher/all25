package org.team100.lib.subsystems.se2;

import org.team100.lib.state.ControlSE2;

/**
 * A planar subsystem controlled by position.
 * 
 * We use this interface for mechanisms whose position can be controlled via
 * outboard closed-loop controllers.
 */
public interface PositionSubsystemSE2 extends SubsystemSE2 {

    /** Position, velocity, and acceleration may all be used. */
    void set(ControlSE2 setpoint);
}