package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * For constructing splines, paths, and trajectories.
 * 
 * This is similar to ModelR3, except it doesn't have the notion of velocity,
 * only direction.
 * 
 * @param pose   location and orientation
 * @param course direction of travel (includes rotation)
 */
public record Pose2dWithDirection(
        Pose2d pose,
        DirectionSE2 course) {

    public Translation2d translation() {
        return pose.getTranslation();
    }

    public Rotation2d heading() {
        return pose.getRotation();
    }

    /** Course without rotation */
    public static Pose2dWithDirection make(Pose2d p, double course) {
        return new Pose2dWithDirection(
                p,
                new DirectionSE2(Math.cos(course), Math.sin(course), 0));
    }

    /**
     * For tank drive, heading and course are the same. This is like the WPI
     * trajectory.
     */
    public static Pose2dWithDirection tank(Pose2d p) {
        Rotation2d r = p.getRotation();
        return new Pose2dWithDirection(
                p, new DirectionSE2(r.getCos(), r.getSin(), 0));
    }
}
