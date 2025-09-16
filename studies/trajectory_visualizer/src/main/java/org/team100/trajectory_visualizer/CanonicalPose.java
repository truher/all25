package org.team100.trajectory_visualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * We often use an enum of canonical poses.
 */
public enum CanonicalPose {
    POSE_ONE(0, 0, 0),
    POSE_TWO(1, 1, 1);

    public final Pose2d pose;

    private CanonicalPose(double x, double y, double r) {
        pose = new Pose2d(x, y, new Rotation2d(r));
    }

}
