package org.team100.lib.geometry;

/**
 * HolonomicPose3d with:
 * 
 * * the spatial rate of change in heading
 * * the spatial rate of change in course
 * * the spatial rate of change in course curvature
 */
public class Pose3dWithMotion {
    /** Pose and course. */
    private final Pose3dWithDirection m_pose;
    /** Change in yaw per meter of motion, rad/m. */
    private final double m_headingYawRate;

    public Pose3dWithMotion(
            Pose3dWithDirection pose,
            double headingYawRate) {
        m_pose = pose;
        m_headingYawRate = headingYawRate;
    }

    public Pose3dWithDirection getPose() {
        return m_pose;
    }

    public double getHeadingYawRateRad_M() {
        return m_headingYawRate;
    }

}
