package org.team100.lib.geometry;

import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A direction (i.e. unit-length vector) in the SE2 manifold,
 * rigid transformations in two dimensions (which have three dimensions).
 * 
 * This is useful for representing spline controls for Pose2d.
 * 
 * It is exactly a unit-length Twist2d.
 */
public class DirectionSE2 {
    private static final boolean DEBUG = false;

    public final double x;
    public final double y;
    public final double theta;

    public DirectionSE2(double px, double py, double ptheta) {
        double h = Math.sqrt(px * px + py * py + ptheta * ptheta);
        x = px / h;
        y = py / h;
        theta = ptheta / h;
    }

    /** Cartesian part of direction, as an old-fashioned Rotation2d */
    public Rotation2d toRotation() {
        return new Rotation2d(x, y);
    }

    /** In the direction of the specified angle in radians, without rotation */
    public static DirectionSE2 irrotational(double rad) {
        return fromDirections(rad, 0);
    }

    /** In the direction of the specified angle, without rotation */
    public static DirectionSE2 irrotational(Rotation2d angle) {
        return fromDirections(angle, 0);
    }

    /** In the direction of the specified angle in radians, while rotating */
    public static DirectionSE2 fromDirections(double rad, double theta) {
        return fromDirections(new Rotation2d(rad), theta);
    }

    /** In the direction of the specified angle, while rotating */
    public static DirectionSE2 fromDirections(Rotation2d angle, double theta) {
        return new DirectionSE2(angle.getCos(), angle.getSin(), theta);
    }

    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof DirectionSE2)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }
        DirectionSE2 other = (DirectionSE2) obj;
        if (!Math100.epsilonEquals(other.x, x)) {
            if (DEBUG)
                System.out.println("wrong x");
            return false;
        }
        if (!Math100.epsilonEquals(other.y, y)) {
            if (DEBUG)
                System.out.println("wrong y");
            return false;
        }
        if (!Math100.epsilonEquals(other.theta, theta)) {
            if (DEBUG)
                System.out.println("wrong theta");
            return false;
        }
        return true;
    }

    @Override
    public String toString() {
        return String.format("%5.3f %5.3f %5.3f", x, y, theta);
    }

}
