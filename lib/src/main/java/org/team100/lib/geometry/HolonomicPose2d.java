package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * For constructing splines, paths, and trajectories.
 * 
 * @param translation
 * @param heading
 * @param course
 */
public record HolonomicPose2d(
        Translation2d translation,
        Rotation2d heading,
        Rotation2d course) {

    public static HolonomicPose2d fromPose(Pose2d p, Rotation2d course) {
        return new HolonomicPose2d(p.getTranslation(), p.getRotation(), course);
    }

    public static HolonomicPose2d make(double x, double y, double heading, double course) {
        return new HolonomicPose2d(
                new Translation2d(x, y), new Rotation2d(heading), new Rotation2d(course));
    }
}
