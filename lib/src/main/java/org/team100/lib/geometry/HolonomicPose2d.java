package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * For constructing splines, paths, and trajectories.
 * 
 * Together, translation and heading constitute Pose2d.
 * 
 * @param translation
 * @param heading     where the front of the robot is facing
 * @param course      direction of travel, including rotation
 * 
 * TODO: use Pose2d here
 */
public record HolonomicPose2d(
        Translation2d translation,
        Rotation2d heading,
        DirectionSE2 course) {

    public Pose2d pose() {
        return new Pose2d(translation, heading);
    }

    public static HolonomicPose2d make(Pose2d p, DirectionSE2 course) {
        return new HolonomicPose2d(p.getTranslation(), p.getRotation(), course);
    }

    /** Course without rotation */
    public static HolonomicPose2d make(Pose2d p, double course) {
        return new HolonomicPose2d(
                p.getTranslation(),
                p.getRotation(),
                new DirectionSE2(Math.cos(course), Math.sin(course), 0));
    }

    /** Course without rotation */
    public static HolonomicPose2d make(double x, double y, double heading, double course) {
        return new HolonomicPose2d(
                new Translation2d(x, y),
                new Rotation2d(heading),
                new DirectionSE2(Math.cos(course), Math.sin(course), 0));
    }

    /** For tank drive, heading and course are the same. */
    public static HolonomicPose2d tank(double x, double y, double heading) {
        return new HolonomicPose2d(
                new Translation2d(x, y),
                new Rotation2d(heading),
                new DirectionSE2(Math.cos(heading), Math.sin(heading), 0));
    }

    /**
     * For tank drive, heading and course are the same. This is like the WPI
     * trajectory.
     */
    public static HolonomicPose2d tank(Pose2d p) {
        Rotation2d r = p.getRotation();
        return new HolonomicPose2d(
                p.getTranslation(),
                r,
                new DirectionSE2(r.getCos(), r.getSin(), 0));
    }
}
