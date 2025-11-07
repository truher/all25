package org.team100.lib.subsystems.lynxmotion_arm;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Workspace pose of each joint. The rotation is the rotation of the parent
 * link, in workspace coordinates, not relative to the previous.
 */
public record LynxArmPose(
        Pose3d p1,
        Pose3d p2,
        Pose3d p3,
        Pose3d p4,
        Pose3d p5,
        Pose3d p6) {
}