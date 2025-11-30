package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

/**
 * For constructing 3d splines, paths, and trajectories.
 * 
 * @param translation
 * @param heading     where we're facing, can include roll
 * @param course      where we're going, ignores roll.
 */
public record HolonomicPose3d(
        Translation3d translation,
        Rotation3d heading,
        Rotation3d course) {

}
