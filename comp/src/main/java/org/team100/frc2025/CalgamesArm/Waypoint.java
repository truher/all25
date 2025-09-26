package org.team100.frc2025.CalgamesArm;

import org.team100.lib.geometry.HolonomicPose2d;

import edu.wpi.first.math.geometry.Rotation2d;

public enum Waypoint {
    FROM_PICK(CanonicalPose.PICK, Rotation2d.kZero),
    TO_PICK(CanonicalPose.PICK, Rotation2d.k180deg),
    /** This is required to make rotation go the right way around */
    GOING_FORWARD(CanonicalPose.BETWEEN, Rotation2d.kCCW_90deg),
    FROM_L4(CanonicalPose.L4, new Rotation2d(Math.toRadians(-90))),
    TO_L4(CanonicalPose.L4, new Rotation2d(Math.toRadians(120))),
    TO_L3(CanonicalPose.L3, new Rotation2d(Math.toRadians(120))),
    TO_L2(CanonicalPose.L2, new Rotation2d(Math.toRadians(120))),
    TO_L1(CanonicalPose.L1, new Rotation2d(Math.toRadians(120)));

    public final HolonomicPose2d hPose;

    private Waypoint(CanonicalPose pose, Rotation2d course) {
        hPose = new HolonomicPose2d(
                pose.pose.getTranslation(),
                pose.pose.getRotation(),
                course);
    }

}
