package org.team100.lib.motion.drivetrain.state;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Meters, radians, and seconds.
 * 
 * SE(2) is the Lie Group of position and orientation transformations in 2d.
 * 
 * This class represents these transformations in the global reference frame,
 * i.e. "field relative" for robot navigation.
 */
public record GlobalSe2Acceleration(double x, double y, double theta) {
    public GlobalSe2Velocity integrate(double dtSec) {
        return new GlobalSe2Velocity(x * dtSec, y * dtSec, theta * dtSec);
    }

    public GlobalSe2Acceleration plus(GlobalSe2Acceleration other) {
        return new GlobalSe2Acceleration(x + other.x, y + other.y, theta + other.theta);
    }

    public GlobalSe2Acceleration minus(GlobalSe2Acceleration other) {
        return new GlobalSe2Acceleration(x - other.x, y - other.y, theta - other.theta);
    }

    public FieldRelativeJerk jerk(GlobalSe2Acceleration previous, double dt) {
        GlobalSe2Acceleration v = minus(previous).div(dt);
        return new FieldRelativeJerk(v.x(), v.y(), v.theta());
    }

    public GlobalSe2Acceleration times(double scalar) {
        return new GlobalSe2Acceleration(x * scalar, y * scalar, theta * scalar);
    }

    public GlobalSe2Acceleration div(double scalar) {
        return new GlobalSe2Acceleration(x / scalar, y / scalar, theta / scalar);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public static GlobalSe2Acceleration diff(
            GlobalSe2Velocity v1,
            GlobalSe2Velocity v2,
            double dtSec) {
        GlobalSe2Velocity dv = v2.minus(v1);
        return new GlobalSe2Acceleration(dv.x() / dtSec, dv.y() / dtSec, dv.theta() / dtSec);
    }

    public GlobalSe2Acceleration clamp(double maxAccel, double maxAlpha) {
        double norm = Math.hypot(x, y);
        double ratio = 1.0;
        if (norm > 1e-3 && norm > maxAccel) {
            ratio = maxAccel / norm;
        }
        return new GlobalSe2Acceleration(ratio * x, ratio * y, MathUtil.clamp(theta, -maxAlpha, maxAlpha));
    }

    public static GlobalSe2Acceleration fromVector(Matrix<N3, N1> v) {
        return new GlobalSe2Acceleration(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(x, y, theta);
    }

    @Override
    public String toString() {
        return String.format("(%5.2f, %5.2f, %5.2f)", x, y, theta);
    }
}
