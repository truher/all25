package org.team100.lib.geometry;

import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;

/** WaypointSE2 with heading rate and curvature. */
public class Pose2dWithMotion {
    private static final boolean DEBUG = false;
    /** Pose and course. */
    private final WaypointSE2 m_waypoint;
    /** Change in heading per meter of motion, rad/m. */
    private final double m_headingRateRad_M;
    /** Change in course per change in distance, rad/m. */
    private final double m_curvatureRad_M;

    /**
     * @param waypoint         location and heading and direction of travel
     * @param headingRateRad_M change in heading, per meter traveled
     * @param curvatureRad_M   change in course per meter traveled.
     */
    public Pose2dWithMotion(
            WaypointSE2 waypoint,
            double headingRateRad_M,
            double curvatureRad_M) {
        m_waypoint = waypoint;
        m_headingRateRad_M = headingRateRad_M;
        m_curvatureRad_M = curvatureRad_M;
    }

    public WaypointSE2 getPose() {
        return m_waypoint;
    }

    /**
     * Heading rate is radians per meter.
     * 
     * If you want radians per second, multiply by velocity (meters per second).
     */
    public double getHeadingRateRad_M() {
        return m_headingRateRad_M;
    }

    /** Radians per meter, which is the reciprocal of the radius. */
    public double getCurvatureRad_M() {
        return m_curvatureRad_M;
    }

    /**
     * Linear interpolation of each component separately.
     * 
     * Not a constant-twist arc.
     */
    public Pose2dWithMotion interpolate(Pose2dWithMotion other, double x) {
        return new Pose2dWithMotion(
                GeometryUtil.interpolate(m_waypoint, other.m_waypoint, x),
                MathUtil.interpolate(m_headingRateRad_M, other.m_headingRateRad_M, x),
                Math100.interpolate(m_curvatureRad_M, other.m_curvatureRad_M, x));
    }

    /**
     * R2 (xy) planar distance only (IGNORES ROTATION) so that planar
     * velocity and curvature works correctly. Not the twist arclength.
     * Not the double-geodesic L2 thing. Just XY translation hypot.
     * 
     * Always non-negative.
     */
    public double distanceCartesian(Pose2dWithMotion other) {
        return Metrics.translationalDistance(m_waypoint.pose(), other.m_waypoint.pose());
    }

    public boolean equals(Object other) {
        if (!(other instanceof Pose2dWithMotion)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }

        Pose2dWithMotion p2dwc = (Pose2dWithMotion) other;
        if (!m_waypoint.equals(p2dwc.m_waypoint)) {
            if (DEBUG)
                System.out.println("wrong waypoint");
            return false;
        }
        if (!Math100.epsilonEquals(m_headingRateRad_M, p2dwc.m_headingRateRad_M)) {
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
                m_waypoint.pose().getTranslation().getX(),
                m_waypoint.pose().getTranslation().getY(),
                m_waypoint.pose().getRotation().getRadians(),
                m_waypoint.course(),
                m_headingRateRad_M,
                m_curvatureRad_M);
    }

}