package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * A direction (i.e. unit-length vector) in the SE2 manifold,
 * rigid transformations in two dimensions (which have three dimensions).
 * 
 * This is useful for representing spline controls for Pose2d.
 * 
 * It is exactly a unit-length Twist2d.
 */
public class DirectionSE2 {
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

    /** Adapter for rotation-free directions */
    public static final DirectionSE2 fromRotation(Rotation2d r) {
        return new DirectionSE2(r.getCos(), r.getSin(), 0);
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof DirectionSE2 other
                && Math.abs(other.x - x) < 1E-9
                && Math.abs(other.y - y) < 1E-9
                && Math.abs(other.theta - theta) < 1E-9;
    }
}
