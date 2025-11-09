package org.team100.lib.geometry;

import java.util.Optional;

import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Represents a state within a 2d holonomic path, i.e. with heading independent
 * from course.
 * 
 * This is a purely spatial construct: for the notion of velocity, you probably
 * want TimedPose.
 */
public class Pose2dWithMotion {
    /** Position and heading. */
    private final Pose2d m_pose;
    /** Field-relative course and the spatial derivative of course. */
    private final MotionDirection m_motionDirection;

    /**
     * Change in course per change in distance, rad/m.
     */
    private final double m_curvatureRad_M;

    /**
     * Change in curvature per meter, rad/m^2
     */
    private final double m_dCurvatureDsRad_M2;

    /**
     * Motionless, no curvature.
     * 
     * @param pose
     */
    public Pose2dWithMotion(Pose2d pose) {
        this(pose, 0, 0, 0, 0, 0);
    }

    /**
     * 
     * @param pose               location and heading of the robot
     * @param dx                 x component of direction
     * @param dy                 y component of direction
     * @param dtheta             change in heading, per meter traveled
     * @param curvatureRad_M     change in course per meter traveled.
     * @param dCurvatureDsRad_M2 acceleration in course per meter traveled squared.
     */
    public Pose2dWithMotion(
            Pose2d pose,
            double dx,
            double dy,
            double dtheta,
            double curvatureRad_M,
            double dCurvatureDsRad_M2) {
        m_pose = pose;
        m_motionDirection = new MotionDirection(dx, dy, dtheta);
        m_curvatureRad_M = curvatureRad_M;
        m_dCurvatureDsRad_M2 = dCurvatureDsRad_M2;
    }

    public Pose2d getPose() {
        return m_pose;
    }

    /** The R3 velocity at the given speed. */
    public GlobalVelocityR3 velocity(double speed) {
        return new GlobalVelocityR3(
                speed * m_motionDirection.dx(),
                speed * m_motionDirection.dy(),
                speed * m_motionDirection.dtheta());
    }

    /** Radians per meter, which is the reciprocal of the radius. */
    public double getCurvature() {
        return m_curvatureRad_M;
    }

    /** Radians per meter squared */
    public double getDCurvatureDs() {
        return m_dCurvatureDsRad_M2;
    }

    public Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    // i think the interpolation of motion direction is invalid; it would yield
    // results not on the unit circle, which makes no sense. maybe the consumers of
    // motion direction always derive the angle anyway?
    public Pose2dWithMotion interpolate(final Pose2dWithMotion other, double x) {
        return new Pose2dWithMotion(getPose().interpolate(other.getPose(), x),
                MathUtil.interpolate(m_motionDirection.dx(), other.m_motionDirection.dx(), x),
                MathUtil.interpolate(m_motionDirection.dy(), other.m_motionDirection.dy(), x),
                MathUtil.interpolate(m_motionDirection.dtheta(), other.m_motionDirection.dtheta(), x),
                Math100.interpolate(getCurvature(), other.getCurvature(), x),
                Math100.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    /**
     * Distance in meters along the arc between the two poses (in either order)
     * produced by a constant twist.
     */
    public double distanceM(final Pose2dWithMotion other) {
        return GeometryUtil.distanceM(getPose(), other.getPose());
    }

    public boolean equals(final Object other) {
        if (!(other instanceof Pose2dWithMotion)) {
            return false;
        }

        Pose2dWithMotion p2dwc = (Pose2dWithMotion) other;
        return getPose().equals(p2dwc.getPose()) &&
                m_motionDirection.equals(p2dwc.m_motionDirection) &&
                Math100.epsilonEquals(getCurvature(), p2dwc.getCurvature()) &&
                Math100.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    public String toString() {
        Pose2d pose = getPose();
        return String.format(
                "x %5.3f, y %5.3f, theta %5.3f, direction %s, curvature %5.3f, dcurvature_ds %5.3f",
                pose.getX(),
                pose.getY(),
                pose.getRotation().getRadians(),
                m_motionDirection,
                getCurvature(),
                getDCurvatureDs());
    }

    /**
     * Direction of the translational part of the motion, or empty if motionless.
     */
    public Optional<Rotation2d> getCourse() {
        return m_motionDirection.course();
    }

    /**
     * Heading rate is radians per meter.
     * 
     * If you want radians per second, multiply by velocity (meters per second).
     */
    public double getHeadingRateRad_M() {
        return m_motionDirection.dtheta();
    }
}