package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.Pose2dWithMotion;

/**
 * Timing constraints govern the assignment of a schedule to a path, creating a
 * trajectory. Different implementations focus on different aspects, e.g.
 * tippiness, wheel slip, etc. Different maneuvers may want different
 * constraints, e.g. some should be slow and precise, others fast and risky.
 */
public interface TimingConstraint {
    /**
     * Maximum allowed pathwise velocity, m/s.
     * 
     * Always positive.
     */
    double maxV(Pose2dWithMotion state);

    /**
     * Maximum allowed pathwise acceleration, m/s^2.
     * 
     * Always positive.
     */
    double maxAccel(Pose2dWithMotion state, double velocityM_S);

    /**
     * Maximum allowed pathwise deceleration, m/s^2.
     * 
     * Always negative.
     */
    double maxDecel(Pose2dWithMotion state, double velocityM_S);
}
