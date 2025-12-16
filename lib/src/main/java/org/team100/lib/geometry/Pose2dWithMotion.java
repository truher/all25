package org.team100.lib.geometry;

import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * WaypointSE2 with:
 * 
 * * the spatial rate of change in heading
 * * the spatial rate of change in course
 * * the spatial rate of change in course curvature
 */
public class Pose2dWithMotion {
    private static final boolean DEBUG = false;
    /** Pose and course. */
    private final WaypointSE2 m_pose;
    /** Change in heading per meter of motion, rad/m. */
    private final double m_headingRate;
    /** Change in course per change in distance, rad/m. */
    private final double m_curvatureRad_M;


    /**
     * @param pose               location and heading and direction of travel
     * @param headingRate        change in heading, per meter traveled
     * @param curvatureRad_M     change in course per meter traveled.
     */
    public Pose2dWithMotion(
            WaypointSE2 pose,
            double headingRate,
            double curvatureRad_M) {
        m_pose = pose;
        m_headingRate = headingRate;
        m_curvatureRad_M = curvatureRad_M;
    }

    public WaypointSE2 getPose() {
        return m_pose;
    }

    // TODO: change to DirectionSE2
    public Rotation2d getCourse() {
        return m_pose.course().toRotation();
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

    /** This no longer uses a constant-twist arc, it's a straight line. */
    public Pose2dWithMotion interpolate(final Pose2dWithMotion other, double x) {
        return new Pose2dWithMotion(
                GeometryUtil.interpolate(m_pose, other.m_pose, x),
                MathUtil.interpolate(m_headingRate, other.m_headingRate, x),
                Math100.interpolate(m_curvatureRad_M, other.m_curvatureRad_M, x));
    }

    /** This no longer uses a constant-twist arc, it's a straight line. */
    public double distanceM(final Pose2dWithMotion other) {
        return m_pose.translation().getDistance(other.m_pose.translation());
    }

    public boolean equals(final Object other) {
        if (!(other instanceof Pose2dWithMotion)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }

        Pose2dWithMotion p2dwc = (Pose2dWithMotion) other;
        if (!m_pose.equals(p2dwc.m_pose)) {
            if (DEBUG)
                System.out.println("wrong waypoint");
            return false;
        }
        if (!Math100.epsilonEquals(m_headingRate, p2dwc.m_headingRate)) {
            if (DEBUG)
                System.out.println("wrong heading rate");
            return false;
        }
        if (!Math100.epsilonEquals(m_curvatureRad_M, p2dwc.m_curvatureRad_M)) {
            if (DEBUG)
                System.out.println("wrong curvature");
            return false;
        }
        return true;
    }

    public String toString() {
        return String.format(
                "x %5.3f, y %5.3f, theta %5.3f, course %s, dtheta %5.3f, curvature %5.3f",
                m_pose.translation().getX(),
                m_pose.translation().getY(),
                m_pose.heading().getRadians(),
                m_pose.course(),
                m_headingRate,
                m_curvatureRad_M);
    }

}