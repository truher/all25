package org.team100.lib.subsystems.prr;

/**
 * Config of the elevator-arm-wrist system.
 * 
 * @param shoulderHeight meters relative to the floor
 * @param shoulderAngle  radians relative to the base of the elevator, i.e. zero
 *                       is straight up
 * @param wristAngle     radians relative to the shoulder, i.e. zero is
 *                       extending in the same direction as the arm.
 */
public record EAWConfig(double shoulderHeight, double shoulderAngle, double wristAngle) {

    /**
     * Apply the velocity for period dt.
     * 
     * x = x0 + v dt
     */
    public EAWConfig integrate(JointVelocities jv, double dt) {
        return new EAWConfig(
                shoulderHeight + jv.elevator() * dt,
                shoulderAngle + jv.shoulder() * dt,
                wristAngle + jv.wrist() * dt);
    }

    /**
     * v = (x - x0) / dt
     */
    public JointVelocities diff(EAWConfig c, double dt) {
        return new JointVelocities(
                (shoulderHeight - c.shoulderHeight) / dt,
                (shoulderAngle - c.shoulderAngle) / dt,
                (wristAngle - c.wristAngle) / dt);
    }

    /** True if any of the axes are NaN */
    public boolean isNaN() {
        return Double.isNaN(shoulderHeight())
                || Double.isNaN(shoulderAngle())
                || Double.isNaN(wristAngle());
    }

    @Override
    public String toString() {
        return String.format("%6.3f %6.3f %6.3f", shoulderHeight, shoulderAngle, wristAngle);
    }
};
