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

    public static final DirectionSE2 TO_X = new DirectionSE2(1, 0, 0);
    public static final DirectionSE2 MINUS_X = new DirectionSE2(-1, 0, 0);
    public static final DirectionSE2 TO_Y = new DirectionSE2(0, 1, 0);
    public static final DirectionSE2 MINUS_Y = new DirectionSE2(0, -1, 0);
    public static final DirectionSE2 SPIN = new DirectionSE2(0, 0, 1);

    /**
     * @param course cartesian course
     * @param theta  direction of rotation
     */
    public static final DirectionSE2 fromDirections(DirectionR2 course, double theta) {
        return new DirectionSE2(course.x, course.y, theta);
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
