package org.team100.lib.trajectory.path.spline;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.HolonomicPose3d;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose3dWithMotion;

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

    private final SplineR1 m_x;
    private final SplineR1 m_y;
    private final SplineR1 m_z;
    private final SplineR1 m_pitch;
    private final SplineR1 m_yaw;
    private final Rotation2d m_pitch0;
    private final Rotation2d m_yaw0;

    public HolonomicSpline3d(HolonomicPose3d p0, HolonomicPose3d p1) {
        this(p0, p1, 1.2, 1.2);
    }

    public HolonomicSpline3d(HolonomicPose3d p0, HolonomicPose3d p1, double mN0, double mN1) {
        double scale0 = mN0 * GeometryUtil.distanceM(p0.translation(), p1.translation());
        double scale1 = mN1 * GeometryUtil.distanceM(p0.translation(), p1.translation());

        Translation3d course0 = new Translation3d(1, 0, 0).rotateBy(p0.course());
        Translation3d course1 = new Translation3d(1, 0, 0).rotateBy(p1.course());

        double x0 = p0.translation().getX();
        double x1 = p1.translation().getX();
        // first derivatives are just the course
        double dx0 = course0.getX() * scale0;
        double dx1 = course1.getX() * scale1;
        // second derivatives are zero at the ends
        double ddx0 = 0;
        double ddx1 = 0;

        double y0 = p0.translation().getY();
        double y1 = p1.translation().getY();
        // first derivatives are just the course
        double dy0 = course0.getY() * scale0;
        double dy1 = course1.getY() * scale1;
        // second derivatives are zero at the ends
        double ddy0 = 0;
        double ddy1 = 0;

        double z0 = p0.translation().getZ();
        double z1 = p1.translation().getZ();
        // first derivatives are just the course
        double dz0 = course0.getZ() * scale0;
        double dz1 = course1.getZ() * scale1;
        // second derivatives are zero at the ends
        double ddz0 = 0;
        double ddz1 = 0;

        m_x = SplineR1.get(x0, x1, dx0, dx1, ddx0, ddx1);
        m_y = SplineR1.get(y0, y1, dy0, dy1, ddy0, ddy1);
        m_z = SplineR1.get(z0, z1, dz0, dz1, ddz0, ddz1);

        m_pitch0 = new Rotation2d(p0.heading().getY());
        m_yaw0 = new Rotation2d(p0.heading().getZ());

        Rotation3d headingDelta = p1.heading().minus(p0.heading());
        double pitchDelta = headingDelta.getY();
        double yawDelta = headingDelta.getZ();

        double dpitch0 = pitchDelta * mN0;
        double dpitch1 = pitchDelta * mN1;
        // second derivatives are zero at the ends
        double ddpitch0 = 0;
        double ddpitch1 = 0;
        m_pitch = SplineR1.get(0.0, pitchDelta, dpitch0, dpitch1, ddpitch0, ddpitch1);

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

    public Pose3dWithMotion getPose3dWithMotion(double p) {
        return null;
        // return new Pose3dWithMotion(
        //         new HolonomicPose3d(
        //                 getPoint(p),
        //                 getHeading(p),
        //                 getCourse(p).orElseThrow()),
        //         getDHeadingDs(p),
        //         getCurvature(p),
        //         getDCurvatureDs(p));
    }

    protected Rotation3d getHeading(double t) {
        return null;
        // return m_r0.rotateBy(Rotation2d.fromRadians(m_theta.getPosition(t)));
    }

}
