package org.team100.lib.geometry;

import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * HolonomicPose2d with:
 * 
 * * the spatial rate of change in heading
 * * the spatial rate of change in course
 * * the spatial rate of change in course curvature
 */
public class Pose2dWithMotion {
    /** Position, heading and course. */
    private final HolonomicPose2d m_pose;
    /** Change in heading per meter of motion, rad/m. */
    private final double m_headingRate;
    /** Change in course per change in distance, rad/m. */
    private final double m_curvatureRad_M;
    /** Change in curvature per meter, rad/m^2 */
    private final double m_dCurvatureDsRad_M2;

    /**
     * @param pose               location and heading of the robot
     * @param course             motion direction, radians
     * @param headingRate        change in heading, per meter traveled
     * @param curvatureRad_M     change in course per meter traveled.
     * @param dCurvatureDsRad_M2 acceleration in course per meter traveled squared.
     */
    public Pose2dWithMotion(
            HolonomicPose2d pose,
            double headingRate,
            double curvatureRad_M,
            double dCurvatureDsRad_M2) {
        m_pose = pose;
        m_headingRate = headingRate;
        m_curvatureRad_M = curvatureRad_M;
        m_dCurvatureDsRad_M2 = dCurvatureDsRad_M2;
    }

    public HolonomicPose2d getPose() {
        return m_pose;
    }

    public Rotation2d getCourse() {
        return m_pose.course();
    }

    /**
     * Heading rate is radians per meter.
     * 
     * If you want radians per second, multiply by velocity (meters per second).
     */
    public double getHeadingRateRad_M() {
        return m_headingRate;
    }

    /** Radians per meter, which is the reciprocal of the radius. */
    public double getCurvature() {
        return m_curvatureRad_M;
    }

    /** Radians per meter squared */
    public double getDCurvatureDs() {
        return m_dCurvatureDsRad_M2;
    }

    /** This no longer uses a constant-twist arc, it's a straight line. */
    public Pose2dWithMotion interpolate(final Pose2dWithMotion other, double x) {
        return new Pose2dWithMotion(
                GeometryUtil.interpolate(m_pose, other.m_pose, x),
                MathUtil.interpolate(m_headingRate, other.m_headingRate, x),
                Math100.interpolate(m_curvatureRad_M, other.m_curvatureRad_M, x),
                Math100.interpolate(m_dCurvatureDsRad_M2, other.m_dCurvatureDsRad_M2, x));
    }

    /** This no longer uses a constant-twist arc, it's a straight line. */
    public double distanceM(final Pose2dWithMotion other) {
        return m_pose.translation().getDistance(other.m_pose.translation());
    }

    public boolean equals(final Object other) {
        if (!(other instanceof Pose2dWithMotion)) {
            return false;
        }

        Pose2dWithMotion p2dwc = (Pose2dWithMotion) other;
        return m_pose.equals(p2dwc.m_pose) &&
                Math100.epsilonEquals(m_headingRate, p2dwc.m_headingRate) &&
                Math100.epsilonEquals(m_curvatureRad_M, p2dwc.m_curvatureRad_M) &&
                Math100.epsilonEquals(m_dCurvatureDsRad_M2, p2dwc.m_dCurvatureDsRad_M2);
    }

    public String toString() {
        return String.format(
                "x %5.3f, y %5.3f, theta %5.3f, course %s, dtheta %5.3f, curvature %5.3f, dcurvature_ds %5.3f",
                m_pose.translation().getX(),
                m_pose.translation().getY(),
                m_pose.heading().getRadians(),
                m_pose.course(),
                m_headingRate,
                m_curvatureRad_M,
                m_dCurvatureDsRad_M2);
    }

}