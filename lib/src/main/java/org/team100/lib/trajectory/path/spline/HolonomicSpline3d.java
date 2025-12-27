package org.team100.lib.trajectory.path.spline;

import java.util.Optional;

import org.team100.lib.geometry.DirectionR3;
import org.team100.lib.geometry.DirectionSE3;
import org.team100.lib.geometry.Pose3dWithDirection;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * Holonomic spline in three dimensions, similar to the 2d version.
 * 
 * Internally this is five one-dimensional splines (x, y, z, phi, theta), with
 * respect to a parameter [0,1].
 * 
 * Elsewhere, we use Rotation3d to describe spherical angles, by applying the
 * rotation to the translation (1,0,0), as Pose3d does. The coordinate system
 * there is "roll pitch yaw" from the reference, not "spherical coordinates"
 * (with a polar angle etc).
 */
public class HolonomicSpline3d {
    private static final boolean DEBUG = false;
    // curvature measurement performance scales with sample count so make it kinda
    // low. most splines go between 0.5 and 5 meters so this is steps of 2 to 20 cm.
    private static final int SAMPLES = 25;

    // these are for position
    private final SplineR1 m_x;
    private final SplineR1 m_y;
    private final SplineR1 m_z;

    // these are for heading
    private final SplineR1 m_roll;
    private final SplineR1 m_pitch;
    private final SplineR1 m_yaw;
    private final Rotation2d m_roll0;
    private final Rotation2d m_pitch0;
    private final Rotation2d m_yaw0;

    public HolonomicSpline3d(Pose3dWithDirection p0, Pose3dWithDirection p1) {
        this(p0, p1, 1.2, 1.2);
    }

    public HolonomicSpline3d(Pose3dWithDirection p0, Pose3dWithDirection p1, double mN0, double mN1) {
        double scale0 = mN0 * p0.translation().getDistance(p1.translation());
        double scale1 = mN1 * p0.translation().getDistance(p1.translation());

        DirectionSE3 course0 = p0.course();
        DirectionSE3 course1 = p1.course();

        double x0 = p0.translation().getX();
        double x1 = p1.translation().getX();
        // first derivatives are just the course
        double dx0 = course0.x * scale0;
        double dx1 = course1.x * scale1;
        // second derivatives are zero at the ends
        double ddx0 = 0;
        double ddx1 = 0;

        double y0 = p0.translation().getY();
        double y1 = p1.translation().getY();
        // first derivatives are just the course
        double dy0 = course0.y * scale0;
        double dy1 = course1.y * scale1;
        // second derivatives are zero at the ends
        double ddy0 = 0;
        double ddy1 = 0;

        double z0 = p0.translation().getZ();
        double z1 = p1.translation().getZ();
        // first derivatives are just the course
        double dz0 = course0.z * scale0;
        double dz1 = course1.z * scale1;
        // second derivatives are zero at the ends
        double ddz0 = 0;
        double ddz1 = 0;

        m_x = SplineR1.get(x0, x1, dx0, dx1, ddx0, ddx1);
        m_y = SplineR1.get(y0, y1, dy0, dy1, ddy0, ddy1);
        m_z = SplineR1.get(z0, z1, dz0, dz1, ddz0, ddz1);

        m_roll0 = new Rotation2d(p0.heading().getX());
        m_pitch0 = new Rotation2d(p0.heading().getY());
        m_yaw0 = new Rotation2d(p0.heading().getZ());

        Rotation3d headingDelta = p1.heading().minus(p0.heading());
        double rollDelta = headingDelta.getX();
        double pitchDelta = headingDelta.getY();
        double yawDelta = headingDelta.getZ();

        // first derivative is the average
        double droll0 = rollDelta * mN0;
        double droll1 = rollDelta * mN1;
        // second derivatives are zero at the ends
        double ddroll0 = 0;
        double ddroll1 = 0;
        m_roll = SplineR1.get(0.0, rollDelta, droll0, droll1, ddroll0, ddroll1);

        // first derivative is the average
        double dpitch0 = pitchDelta * mN0;
        double dpitch1 = pitchDelta * mN1;
        // second derivatives are zero at the ends
        double ddpitch0 = 0;
        double ddpitch1 = 0;
        m_pitch = SplineR1.get(0.0, pitchDelta, dpitch0, dpitch1, ddpitch0, ddpitch1);

        // first derivative is the average
        double dyaw0 = yawDelta * mN0;
        double dyaw1 = yawDelta * mN1;
        // second derivatives are zero at the ends
        double ddyaw0 = 0;
        double ddyaw1 = 0;
        m_yaw = SplineR1.get(0.0, yawDelta, dyaw0, dyaw1, ddyaw0, ddyaw1);

    }

    /**
     * Cartesian coordinate in meters.
     * 
     * @param t ranges from 0 to 1
     * @return the point on the spline for that t value
     */
    protected Translation3d getPoint(double t) {
        return new Translation3d(x(t), y(t), z(t));
    }

    double x(double t) {
        return m_x.getPosition(t);
    }

    double y(double t) {
        return m_y.getPosition(t);
    }

    double z(double t) {
        return m_z.getPosition(t);
    }

    // public Pose3dWithMotion getPose3dWithMotion(double p) {
    // return new Pose3dWithMotion(
    // new HolonomicPose3d(
    // getPoint(p),
    // getHeading(p),
    // getCourse(p).orElseThrow()),
    // getDHeadingDs(p),
    // getCurvature(p),
    // getDCurvatureDs(p));
    // }

    public Optional<DirectionR3> getCourse(double t) {
        double dx = dx(t);
        double dy = dy(t);
        double dz = dz(t);
        if (Math100.epsilonEquals(dx, 0.0)
                && Math100.epsilonEquals(dy, 0.0)
                && Math100.epsilonEquals(dz, 0.0)) {
            // rotation below would be garbage so give up
            return Optional.empty();
        }
        return Optional.of(new DirectionR3(dx, dy, dz));
    }

    protected Rotation3d getHeading(double t) {
        return null;
        // return m_r0.rotateBy(Rotation2d.fromRadians(m_theta.getPosition(t)));
    }

    double dx(double t) {
        return m_x.getVelocity(t);
    }

    double dy(double t) {
        return m_y.getVelocity(t);
    }

    double dz(double t) {
        return m_z.getVelocity(t);
    }

    /**
     * Velocity is the change in position per parameter, p: ds/dp (meters per p).
     * Since p is not time, it is not "velocity" in the usual sense.
     */
    protected double getVelocity(double t) {
        double dx = dx(t);
        double dy = dy(t);
        double dz = dy(t);
        return Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

}
