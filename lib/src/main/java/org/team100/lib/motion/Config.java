package org.team100.lib.motion;

import org.team100.lib.motion.kinematics.JointVelocities;

//should be renamed to match the EWAK Convention (elevator wrist arm kinematics)
/**
 * @param shoulderHeight meters relative to the floor
 * @param shoulderAngle  radians relative to the base of the elevator, i.e. zero
 *                       is straight up
 * @param wristAngle     radians relative to the shoulder, i.e. zero is
 *                       extending in the same direction as the arm.
 */
public record Config(double shoulderHeight, double shoulderAngle, double wristAngle) {

    /**
     * Apply the velocity for period dt.
     * 
     * x = x0 + v dt
     */
    public Config integrate(JointVelocities jv, double dt) {
        return new Config(
                shoulderHeight + jv.elevator() * dt,
                shoulderAngle + jv.shoulder() * dt,
                wristAngle + jv.wrist() * dt);
    }

    /**
     * v = (x - x0) / dt
     */
    public JointVelocities diff(Config c, double dt) {
        return new JointVelocities(
                (c.shoulderHeight - shoulderHeight) / dt,
                (c.shoulderAngle - shoulderAngle) / dt,
                (c.wristAngle - wristAngle) / dt);
    }

    public boolean isNaN() {
        return Double.isNaN(shoulderHeight())
                || Double.isNaN(shoulderAngle())
                || Double.isNaN(wristAngle());
    }
};
