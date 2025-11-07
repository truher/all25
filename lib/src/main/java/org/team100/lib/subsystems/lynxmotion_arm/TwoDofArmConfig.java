package org.team100.lib.subsystems.lynxmotion_arm;

import edu.wpi.first.math.MathUtil;

/**
 * Configuration of the joints.
 * 
 * @param q1 Proximal radians, [-pi, pi]. Zero is along the x axis, positive is
 *           counterclockwise.
 * 
 * @param q2 Distal radians, [-pi, pi]. Zero is along the parent link, positive
 *           is counterclockwise. Because the "elbow" is always "up", this angle
 *           is always between pi and 2pi.
 */
public record TwoDofArmConfig(double q1, double q2) {
    public TwoDofArmConfig(double q1, double q2) {
        this.q1 = MathUtil.angleModulus(q1);
        this.q2 = MathUtil.angleModulus(q2);
    }
}