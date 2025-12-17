package org.team100.lib.trajectory.path.spline;

import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.WaypointSE2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Holonomic spline.
 * 
 * Internally this is three one-dimensional splines (x, y, heading), with
 * respect to a parameter [0,1].
 * 
 * If you don't care about rotation, just pass zero.
 * 
 * Note that some nonholonomic spline consumers assume that dx carries all the
 * motion; that's not true here.
 * 
 * This happily produces "splines" with sharp corners, if the segment
 * derivatives don't match.
 */
public class HolonomicSpline {
    private static final boolean DEBUG = false;

    private final SplineR1 m_x;
    private final SplineR1 m_y;
    private final SplineR1 m_heading;
    /**
     * Offset for heading spline: the heading spline doesn't include the
     * starting point in order to correctly handle wrapping.
     */
    private final Rotation2d m_heading0;

    /**
     * Specify the magic number you want: this scales the derivatives at the
     * endpoints, i.e. how "strongly" the derivative affects the curve. High magic
     * number means low curvature at the endpoint.
     * 
     * Previously, the theta endpoint derivatives were the average rate, which
     * yielded paths with a lot of rotation at the last second. Typically this isn't
     * what you want: you're approaching some target, and rotation is disruptive to
     * everything: vision, actuation, everything.
     *
     * Instead, we now use the "course" to specify the whole SE(2) direction, so if
     * you want rotation at the end, you can say that, and if you want no rotation
     * at the end, you can say that too.
     * 
     * All second derivatives are zero, and we don't try to change this anymore with
     * optimization. Optimization just doesn't help very much, and it's a pain when
     * it behaves strangely.
     * 
     * @param p0 starting pose
     * @param p1 ending pose
     */
    public HolonomicSpline(WaypointSE2 p0, WaypointSE2 p1) {
        // Distance metric includes both translation and rotation. This is not
        // the geodesic distance, which is zero for spin-in-place. It's just the
        // L2 norm for all three dimensions.
        double distance = GeometryUtil.doubleGeodesicDistance(p0.pose(), p1.pose());
        if (DEBUG)
            System.out.printf("distance %f\n", distance);
        double scale0 = p0.scale() * distance;
        double scale1 = p1.scale() * distance;

        if (DEBUG) {
            System.out.printf("scale %f %f\n", scale0, scale1);
        }

        // Endpoints:
        double x0 = p0.pose().getTranslation().getX();
        double x1 = p1.pose().getTranslation().getX();
        double y0 = p0.pose().getTranslation().getY();
        double y1 = p1.pose().getTranslation().getY();
        // To avoid 180 degrees, heading uses an offset
        m_heading0 = p0.pose().getRotation();
        double delta = p1.pose().getRotation().minus(p0.pose().getRotation()).getRadians();

        // First derivatives are the course:
        double dx0 = p0.course().x * scale0;
        double dx1 = p1.course().x * scale1;
        double dy0 = p0.course().y * scale0;
        double dy1 = p1.course().y * scale1;
        double dtheta0 = p0.course().theta * scale0;
        double dtheta1 = p1.course().theta * scale1;

        // Second derivatives are zero:
        double ddx0 = 0;
        double ddx1 = 0;
        double ddy0 = 0;
        double ddy1 = 0;
        double ddtheta0 = 0;
        double ddtheta1 = 0;

        m_x = SplineR1.get(x0, x1, dx0, dx1, ddx0, ddx1);
        m_y = SplineR1.get(y0, y1, dy0, dy1, ddy0, ddy1);
        m_heading = SplineR1.get(0.0, delta, dtheta0, dtheta1, ddtheta0, ddtheta1);
    }

    @Override
    public String toString() {
        return "HolonomicSpline [m_x=" + m_x + ", m_y=" + m_y + ", m_theta=" + m_heading + ", m_r0=" + m_heading0 + "]";
    }

    /**
     * TODO: eliminate the waypoint here, for sure eliminate the scale.
     * 
     * @param p [0,1]
     */
    public Pose2dWithMotion getPose2dWithMotion(double p) {
        return new Pose2dWithMotion(
                new WaypointSE2(getPose2d(p), getCourse(p), 1), // <<< TODO: remove the "1"
                getDHeadingDs(p),
                getCurvature(p));
    }

    /**
     * Direction of motion in SE(2). Includes both cartesian dimensions and also the
     * rotation dimension. This is exactly a unit-length twist in the motion
     * direction.
     */
    private DirectionSE2 getCourse(double p) {
        double dx = dx(p);
        double dy = dy(p);
        double dtheta = dtheta(p);
        return new DirectionSE2(dx, dy, dtheta);
    }

    public Pose2d getPose2d(double p) {
        return new Pose2d(new Translation2d(x(p), y(p)), getHeading(p));
    }

    ////////////////////////////////////////////////////////////////////////

    private double getDHeading(double t) {
        return m_heading.getVelocity(t);
    }

    /**
     * Change in heading per distance traveled, i.e. spatial change in heading.
     * dtheta/ds (radians/meter).
     */
    private double getDHeadingDs(double p) {
        return getDHeading(p) / getVelocity(p);
    }

    /** x at p */
    double x(double p) {
        return m_x.getPosition(p);
    }

    /** y at p */
    double y(double p) {
        return m_y.getPosition(p);
    }

    /** heading at p */
    private Rotation2d getHeading(double t) {
        double headingFromZero = m_heading.getPosition(t);
        return m_heading0.rotateBy(Rotation2d.fromRadians(headingFromZero));
    }

    /** dx/dp */
    double dx(double p) {
        return m_x.getVelocity(p);
    }

    /** dy/dp */
    double dy(double p) {
        return m_y.getVelocity(p);
    }

    /** dheading/dp */
    double dtheta(double p) {
        return m_heading.getVelocity(p);
    }

    /** d^2x/dp^2 */
    double ddx(double p) {
        return m_x.getAcceleration(p);
    }

    /** d^2y/dp^2 */
    double ddy(double p) {
        return m_y.getAcceleration(p);
    }

    /** d^2heading/dp^2 */
    double ddtheta(double p) {
        return m_heading.getAcceleration(p);
    }

    /**
     * Velocity is the change in position per parameter, p: ds/dp (meters per p).
     * Since p is not time, it is not "velocity" in the usual sense.
     */
    private double getVelocity(double t) {
        return Math.hypot(dx(t), dy(t));
    }

    /**
     * Curvature is the change in motion direction per distance traveled.
     * rad/m.
     * Note the denominator is distance in this case, not the parameter, p.
     * but the argument to this function *is* the parameter, p. :-)
     */
    private double getCurvature(double t) {
        double dx = dx(t);
        double dy = dy(t);
        double ddx = ddx(t);
        double ddy = ddy(t);
        return (dx * ddy - ddx * dy) / ((dx * dx + dy * dy) * Math.sqrt((dx * dx + dy * dy)));
    }
}