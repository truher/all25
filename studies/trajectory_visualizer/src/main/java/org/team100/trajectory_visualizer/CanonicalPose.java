package org.team100.trajectory_visualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * We often use an enum of canonical poses.
 */
public enum CanonicalPose {
    HOME(1, 0, 0),
    PICK(0.1, -0.6, Rotation2d.kCW_90deg.getRadians()),
    /** This is required to make rotation go the right way around */
    BETWEEN(1, 0, 0),
    L4(1.9, 0.5, Math.toRadians(150)),
    L3(1.2, 0.5, Math.toRadians(120)),
    L2(0.85, 0.5, Math.toRadians(120)),
    L1(0.5, 0.5, Math.toRadians(90));

    public final Pose2d pose;

    private CanonicalPose(double x, double y, double r) {
        pose = new Pose2d(x, y, new Rotation2d(r));
    }

}
