package org.team100.lib.motion.drivetrain.state;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Just like ChassisSpeeds, but field-relative, to avoid mixing them up.
 * 
 * Units are always meters per second and radians per second, never anything
 * else.
 * 
 * SE(2) is the Lie Group of position and orientation transformations in 2d.
 * 
 * This class represents these transformations in the global reference frame,
 * i.e. "field relative" for robot navigation.
 */
public record GlobalSe2Velocity(double x, double y, double theta) {

    public static final GlobalSe2Velocity ZERO = new GlobalSe2Velocity(0, 0, 0);

    public static GlobalSe2Velocity zero() {
        return ZERO;
    }

    public static GlobalSe2Velocity velocity(Pose2d start, Pose2d end, double dt) {
        FieldRelativeDelta d = FieldRelativeDelta.delta(start, end);
        return new GlobalSe2Velocity(d.getX(), d.getY(), d.getRotation().getRadians()).div(dt);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public GlobalSe2Velocity normalize() {
        double norm = norm();
        if (norm < 1e-6)
            return zero();
        return new GlobalSe2Velocity(x, y, theta).times(1.0 / norm);
    }

    /** Field-relative course, or empty if slower than 1 micron/sec. */
    public Optional<Rotation2d> angle() {
        if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6)
            return Optional.empty();
        return Optional.of(new Rotation2d(x, y));
    }

    public GlobalSe2Velocity plus(GlobalSe2Velocity other) {
        return new GlobalSe2Velocity(x + other.x, y + other.y, theta + other.theta);
    }

    /** The return type here isn't really right. */
    public GlobalSe2Velocity minus(GlobalSe2Velocity other) {
        return new GlobalSe2Velocity(x - other.x, y - other.y, theta - other.theta);
    }

    public GlobalSe2Acceleration accel(GlobalSe2Velocity previous, double dt) {
        GlobalSe2Velocity v = minus(previous).div(dt);
        return new GlobalSe2Acceleration(v.x(), v.y(), v.theta());
    }

    public GlobalSe2Velocity times(double scalar) {
        return new GlobalSe2Velocity(x * scalar, y * scalar, theta * scalar);
    }

    public GlobalSe2Velocity div(double scalar) {
        return new GlobalSe2Velocity(x / scalar, y / scalar, theta / scalar);
    }

    public GlobalSe2Velocity times(double cartesian, double angular) {
        return new GlobalSe2Velocity(x * cartesian, y * cartesian, theta * angular);
    }

    /** Dot product of translational part. */
    public double dot(GlobalSe2Velocity other) {
        return x * other.x + y * other.y;
    }

    /** Stopping distance at the specified acceleration. */
    public Translation2d stopping(double accel) {
        double speed = norm();
        double decelTime = speed / accel;
        // the unit here is wrong
        GlobalSe2Velocity dx = normalize().times(0.5 * speed * decelTime);
        return new Translation2d(dx.x, dx.y);
    }

    public GlobalSe2Velocity clamp(double maxVelocity, double maxOmega) {
        double norm = Math.hypot(x, y);
        double ratio = 1.0;
        if (norm > 1e-3 && norm > maxVelocity) {
            ratio = maxVelocity / norm;
        }
        return new GlobalSe2Velocity(ratio * x, ratio * y, MathUtil.clamp(theta, -maxOmega, maxOmega));
    }

    @Override
    public String toString() {
        return String.format("(%5.2f, %5.2f, %5.2f)", x, y, theta);
    }

    public static GlobalSe2Velocity fromVector(Vector<N3> v) {
        return new GlobalSe2Velocity(v.get(0), v.get(1), v.get(2));
    }

    public static GlobalSe2Velocity fromVector(Matrix<N3, N1> v) {
        return new GlobalSe2Velocity(v.get(0, 0), v.get(1, 0), v.get(2, 0));
    }

    public Vector<N3> toVector() {
        return VecBuilder.fill(x, y, theta);
    }
}