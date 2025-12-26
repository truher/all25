package org.team100.lib.geometry;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;

/** Velocity in R2, companion to Translation2d. */
public record GlobalVelocityR2(double x, double y) {
    public static GlobalVelocityR2 ZERO = new GlobalVelocityR2(0, 0);

    public static GlobalVelocityR2 fromPolar(Rotation2d angle, double speed) {
        return new GlobalVelocityR2(speed * angle.getCos(), speed * angle.getSin());
    }

    /** Pick up the translation component of v. */
    public static GlobalVelocityR2 fromSe2(VelocitySE2 v) {
        return new GlobalVelocityR2(v.x(), v.y());
    }

    public GlobalVelocityR2 plus(GlobalVelocityR2 other) {
        return new GlobalVelocityR2(x + other.x, y + other.y);
    }

    public GlobalVelocityR2 minus(GlobalVelocityR2 other) {
        return new GlobalVelocityR2(x - other.x, y - other.y);
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    /** Field-relative course, or empty if slower than 1 micron/sec. */
    public Optional<Rotation2d> angle() {
        if (norm() < 1e-6)
            return Optional.empty();
        return Optional.of(new Rotation2d(x, y));
    }

    /** Dot product. */
    public double dot(GlobalVelocityR2 other) {
        return x * other.x + y * other.y;
    }

    /** Dot product with translation. */
    public double dot(Translation2d other) {
        return x * other.getX() + y * other.getY();
    }

    public Translation2d integrate(Translation2d start, double dt) {
        return new Translation2d(start.getX() + x * dt, start.getY() + y * dt);
    }

    public static GlobalVelocityR2 fromVector(Vector<N3> v) {
        return new GlobalVelocityR2(v.get(0), v.get(1));
    }

    public static GlobalVelocityR2 fromVector(Matrix<N3, N1> v) {
        return new GlobalVelocityR2(v.get(0, 0), v.get(1, 0));
    }

    public Vector<N2> toVector() {
        return VecBuilder.fill(x, y);
    }
}
