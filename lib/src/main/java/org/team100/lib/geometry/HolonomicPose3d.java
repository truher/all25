package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * For constructing 3d splines, paths, and trajectories.
 * 
 * Together, translation and heading constitude Pose3d.
 * 
 * @param translation
 * @param heading     where we're facing, can include roll
 * @param course      where we're going
 */
public record HolonomicPose3d(
        Translation3d translation,
        Rotation3d heading,
        DirectionR3 course) {

    public Pose3d pose() {
        return new Pose3d(translation, heading);
    }

}
