package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * For constructing 3d splines, paths, and trajectories.
 * 
 * @param pose   location and orientation
 * @param course direction of travel (including rotation)
 */
public record Pose3dWithDirection(
        Pose3d pose,
        DirectionSE3 course) {

    public Translation3d translation() {
        return pose.getTranslation();
    }

    public Rotation3d heading() {
        return pose.getRotation();
    }
}
