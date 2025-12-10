package org.team100.lib.subsystems.r3;

import org.team100.lib.geometry.GlobalVelocityR3;

/**
 * A subsystem with three independent dimensions, controlled by velocity.
 * 
 * We use this interface for robot movement on the floor, where the three
 * dimensions are the dimensions of Pose2d: x, y, and theta.
 */
public interface VelocitySubsystemR3 extends SubsystemR3 {

    /**
     * No scaling or filtering.
     * 
     * @param nextV for the next timestep.
     */
    void setVelocity(GlobalVelocityR3 nextV);
}