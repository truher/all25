package org.team100.lib.trajectory.path.spline;

import java.util.Optional;

import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.Pose2dWithDirection;
import org.team100.lib.geometry.Pose2dWithMotion;

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
    // curvature measurement performance scales with sample count so make it kinda
    // low. most splines go between 0.5 and 5 meters so this is steps of 2 to 20 cm.
    private static final int SAMPLES = 25;

    private final SplineR1 m_x;
    private final SplineR1 m_y;
    private final SplineR1 m_heading;
    /**
     * Offset for heading spline: the heading spline doesn't include the
     * starting point in order to correctly handle wrapping.
     */
    private final Rotation2d m_heading0;

    /**
     * The theta endpoint derivative is just the average theta rate, which is new,
     * it used to be zero.
     */
    public HolonomicSpline(Pose2dWithDirection p0, Pose2dWithDirection p1) {
        this(p0, p1, 1.2, 1.2);
    }

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
     * @param p0  starting pose
     * @param p1  ending pose
     * @param mN0 starting magic number
     * @param mN1 ending magic number
     */
    public HolonomicSpline(Pose2dWithDirection p0, Pose2dWithDirection p1, double mN0, double mN1) {
        // Distance metric includes both translation and rotation. This is not
        // the geodesic distance, which is zero for spin-in-place. It's just the
        // L2 norm for all three dimensions.
        double distance = GeometryUtil.doubleGeodesicDistance(p0.pose(), p1.pose());
        if (DEBUG)
            System.out.printf("distance %f\n", distance);
        double scale0 = mN0 * distance;
        double scale1 = mN1 * distance;

        if (DEBUG) {
            System.out.printf("scale %f %f\n", scale0, scale1);
        }

        // Endpoints:
        double x0 = p0.translation().getX();
        double x1 = p1.translation().getX();
        double y0 = p0.translation().getY();
        double y1 = p1.translation().getY();
        // To avoid 180 degrees, heading uses an offset
        m_heading0 = p0.heading();
        double delta = p1.heading().minus(p0.heading()).getRadians();

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
     * @param p [0,1]
     */
    public Pose2dWithMotion getPose2dWithMotion(double p) {
        return new Pose2dWithMotion(
                new Pose2dWithDirection(
                        new Pose2d(getPoint(p), getHeading(p)),
                        getCourse(p)),
                getDHeadingDs(p),
                getCurvature(p),
                getDCurvatureDs(p));
    }

    /** So we can see it */
    public void printSamples() {
        System.out.println("p, x, heading, heading rate");
        for (double p = 0; p < 1; p += 0.05) {
            Pose2dWithMotion pwm = getPose2dWithMotion(p);
            System.out.printf("%f, %f, %f, %f\n",
                    p,
                    pwm.getPose().pose().getX(),
                    pwm.getPose().heading().getRadians(),
                    pwm.getHeadingRateRad_M());
        }
    }

    /**
     * Course is the direction of motion in SE(2), which means it includes both
     * cartesian dimensions and also the rotation dimension.
     */
    public DirectionSE2 getCourse(double t) {
        double dx = dx(t);
        double dy = dy(t);
        double dtheta = dtheta(t);
        return new DirectionSE2(dx, dy, dtheta);
    }

    public Pose2d getPose2d(double p) {
        return new Pose2d(getPoint(p), getHeading(p));
    }

    ////////////////////////////////////////////////////////////////////////

    protected Rotation2d getHeading(double t) {
        return m_heading0.rotateBy(Rotation2d.fromRadians(m_heading.getPosition(t)));
    }

    protected double getDHeading(double t) {
        return m_heading.getVelocity(t);
    }

    /**
     * Change in heading per distance traveled, i.e. spatial change in heading.
     * dtheta/ds (radians/meter).
     */
    private double getDHeadingDs(double p) {
        return getDHeading(p) / getVelocity(p);
    }

    /**
     * DCurvatureDs is the change in curvature per distance traveled, i.e. the
     * "spatial change in curvature"
     * 
     * dk/dp / ds/dp = dk/ds
     * rad/mp / m/p = rad/m^2
     */
    private double getDCurvatureDs(double p) {
        return getDCurvature(p) / getVelocity(p);
    }

    /** Returns pose in the nonholonomic sense, where the rotation is the course */
    Optional<Pose2d> getStartPose() {
        double dx = dx(0);
        double dy = dy(0);
        if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
            // rotation below would be garbage, so give up.
            return Optional.empty();
        }
        return Optional.of(new Pose2d(
                getPoint(0),
                new Rotation2d(dx, dy)));
    }

    /** Returns pose in the nonholonomic sense, where the rotation is the course */
    Optional<Pose2d> getEndPose() {
        double dx = dx(1);
        double dy = dy(1);
        if (Math.abs(dx) < 1e-6 && Math.abs(dy) < 1e-6) {
            // rotation below would be garbage, so give up.
            return Optional.empty();
        }
        return Optional.of(new Pose2d(
                getPoint(1),
                new Rotation2d(dx, dy)));
    }

    /**
     * Cartesian coordinate in meters.
     * 
     * @param t ranges from 0 to 1
     * @return the point on the spline for that t value
     */
    protected Translation2d getPoint(double t) {
        return new Translation2d(x(t), y(t));
    }

    double x(double t) {
        return m_x.getPosition(t);
    }

    double y(double t) {
        return m_y.getPosition(t);
    }

    double theta(double t) {
        return getHeading(t).getRadians();
    }

    double dx(double t) {
        return m_x.getVelocity(t);
    }

    double dy(double t) {
        return m_y.getVelocity(t);
    }

    double dtheta(double t) {
        return m_heading.getVelocity(t);
    }

    double ddx(double t) {
        return m_x.getAcceleration(t);
    }

    double ddy(double t) {
        return m_y.getAcceleration(t);
    }

    double ddtheta(double t) {
        return m_heading.getAcceleration(t);
    }

    double dddx(double t) {
        return m_x.getJerk(t);
    }

    double dddy(double t) {
        return m_y.getJerk(t);
    }

    double dddtheta(double t) {
        return m_heading.getJerk(t);
    }

    /**
     * Velocity is the change in position per parameter, p: ds/dp (meters per p).
     * Since p is not time, it is not "velocity" in the usual sense.
     */
    protected double getVelocity(double t) {
        return Math.hypot(dx(t), dy(t));
    }

    /**
     * Curvature is the change in motion direction per distance traveled.
     * rad/m.
     * Note the denominator is distance in this case, not the parameter, p.
     * but the argument to this function *is* the parameter, p. :-)
     */
    protected double getCurvature(double t) {
        double dx = dx(t);
        double dy = dy(t);
        double ddx = ddx(t);
        double ddy = ddy(t);
        return (dx * ddy - ddx * dy) / ((dx * dx + dy * dy) * Math.sqrt((dx * dx + dy * dy)));
    }

    /**
     * DCurvature is the change in curvature per change in p.
     * dk/dp (rad/m per p)
     * If you want change in curvature per meter, use getDCurvatureDs.
     */
    protected double getDCurvature(double t) {
        double dx = dx(t);
        double dy = dy(t);
        double dx2dy2 = (dx * dx + dy * dy);
        double ddx = ddx(t);
        double ddy = ddy(t);
        double dddx = dddx(t);
        double dddy = dddy(t);
        double num = (dx * dddy - dddx * dy) * dx2dy2
                - 3 * (dx * ddy - ddx * dy) * (dx * ddx + dy * ddy);
        return num / (dx2dy2 * dx2dy2 * Math.sqrt(dx2dy2));
    }

    double dCurvature2(double t) {
        double dx = dx(t);
        double dy = dy(t);
        double dx2dy2 = (dx * dx + dy * dy);
        if (dx2dy2 == 0)
            throw new IllegalArgumentException();
        double ddx = ddx(t);
        double ddy = ddy(t);
        double dddx = dddx(t);
        double dddy = dddy(t);
        double num = (dx * dddy - dddx * dy) * dx2dy2
                - 3 * (dx * ddy - ddx * dy) * (dx * ddx + dy * ddy);
        return num * num / (dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2 * dx2dy2);
    }

    /** integrate curvature over the length of the spline. */
    double maxCurvature() {
        double dt = 1.0 / SAMPLES;
        double maxC = 0;
        for (double t = 0; t < 1.0; t += dt) {
            maxC = Math.max(maxC, getCurvature(t));
        }
        return maxC;
    }

    /** integrate curvature over the length of the spline. */
    double sumCurvature() {
        double dt = 1.0 / SAMPLES;
        double sum = 0;
        for (double t = 0; t < 1.0; t += dt) {
            sum += (dt * getCurvature(t));
        }
        return sum;
    }

    /**
     * @return integral of dCurvature^2 over the length of the spline
     */
    double sumDCurvature2() {
        double dt = 1.0 / SAMPLES;
        double sum = 0;
        for (double t = 0; t < 1.0; t += dt) {
            sum += (dt * dCurvature2(t));
        }
        return sum;
    }

}