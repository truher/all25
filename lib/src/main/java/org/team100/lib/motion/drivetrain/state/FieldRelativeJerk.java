package org.team100.lib.motion.drivetrain.state;

public record FieldRelativeJerk(double x, double y, double theta) {
    public double norm() {
        return Math.hypot(x, y);
    }

    public FieldRelativeJerk times(double scalar) {
        return new FieldRelativeJerk(x * scalar, y * scalar, theta * scalar);
    }

    public GlobalSe2Acceleration integrate(double dtSec) {
        return new GlobalSe2Acceleration(x * dtSec, y * dtSec, theta * dtSec);
    }
}
