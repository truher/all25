package org.team100.lib.geometry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * The second derivative of Pose2d with respect to time.
 * 
 * Units are meters, radians, and seconds.
 * 
 * This uses the R3 tangent space, not the SE(2) manifold; see README.md for
 * details.
 */
public record AccelerationSE2(double x, double y, double theta) {
    public VelocitySE2 integrate(double dtSec) {
        return new VelocitySE2(x * dtSec, y * dtSec, theta * dtSec);
    }

    public AccelerationSE2 plus(AccelerationSE2 other) {
        return new AccelerationSE2(x + other.x, y + other.y, theta + other.theta);
    }

    public AccelerationSE2 minus(AccelerationSE2 other) {
        return new AccelerationSE2(x - other.x, y - other.y, theta - other.theta);
    }

    public AccelerationSE2 times(double scalar) {
        return new AccelerationSE2(x * scalar, y * scalar, theta * scalar);
    }

    public AccelerationSE2 div(double scalar) {
        return new AccelerationSE2(x / scalar, y / scalar, theta / scalar);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public static AccelerationSE2 diff(
            VelocitySE2 v1,
            VelocitySE2 v2,
            double dtSec) {
        VelocitySE2 dv = v2.minus(v1);
        return new AccelerationSE2(dv.x() / dtSec, dv.y() / dtSec, dv.theta() / dtSec);
    }

    public AccelerationSE2 clamp(double maxAccel, double maxAlpha) {
        double norm = Math.hypot(x, y);
        double ratio = 1.0;
        if (norm > 1e-3 && norm > maxAccel) {
            ratio = maxAccel / norm;
        }
        return new AccelerationSE2(ratio * x, ratio * y, MathUtil.clamp(theta, -maxAlpha, maxAlpha));
    }

    public static AccelerationSE2 fromVector(Matrix<N3, N1> v) {
        return new AccelerationSE2(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(x, y, theta);
    }

    @Override
    public String toString() {
        return String.format("(%5.2f, %5.2f, %5.2f)", x, y, theta);
    }
}
