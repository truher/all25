package org.team100.lib.geometry;

/**
 * HolonomicPose3d with:
 * 
 * * the spatial rate of change in heading
 * * the spatial rate of change in course
 * * the spatial rate of change in course curvature
 */
public class Pose3dWithMotion {
    /** Position, heading and course. */
    private final HolonomicPose3d m_pose;
    /** Change in yaw per meter of motion, rad/m. */
    private final double m_headingYawRate;

    public Pose3dWithMotion(
            HolonomicPose3d pose,
            double headingYawRate) {
        m_pose = pose;
        m_headingYawRate = headingYawRate;
    }

    public HolonomicPose3d getPose() {
        return m_pose;
    }

    public double getHeadingYawRateRad_M() {
        return m_headingYawRate;
    }

}
