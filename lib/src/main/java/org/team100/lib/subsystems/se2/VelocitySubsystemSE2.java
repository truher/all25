package org.team100.lib.subsystems.se2;

import org.team100.lib.geometry.VelocitySE2;

/**
 * A planar subsystem controlled by velocity.
 * 
 * We use this interface for robot movement on the floor, where the three
 * dimensions are the dimensions of Pose2d: x, y, and theta.
 */
public interface VelocitySubsystemSE2 extends SubsystemSE2 {

    /**
     * No scaling or filtering.
     * 
     * @param nextV for the next timestep.
     */
    void setVelocity(VelocitySE2 nextV);
}