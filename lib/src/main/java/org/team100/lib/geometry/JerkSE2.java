package org.team100.lib.geometry;

/**
 * Third time-derivative of Pose2d.
 * 
 * Uses R3 tangent, not SE(2) manifold.
 */
public record JerkSE2(double x, double y, double theta) {
    public double norm() {
        return Math.hypot(x, y);
    }

    public JerkSE2 times(double scalar) {
        return new JerkSE2(x * scalar, y * scalar, theta * scalar);
    }

    public AccelerationSE2 integrate(double dtSec) {
        return new AccelerationSE2(x * dtSec, y * dtSec, theta * dtSec);
    }
}
