package org.team100.frc2025.CalgamesArm;

import org.team100.lib.geometry.HolonomicPose2d;

import edu.wpi.first.math.geometry.Rotation2d;

public enum Waypoint {
    /** This is required to make rotation go the right way around */

    //moving from wherever
    // FROM_WHEREVER()
    //pick
    FROM_PICK(CanonicalPose.PICK, Rotation2d.kZero),
    TO_PICK(CanonicalPose.PICK, Rotation2d.k180deg),

    //algae pick
    TO_APICK(CanonicalPose.APICK, new Rotation2d(Math.toRadians(110))), //goal rotation is 110-120 from vert 
    FROM_APICK(CanonicalPose.APICK, new Rotation2d(Math.toRadians(-70))),

    //station pickup
    TO_STATION(CanonicalPose.STATION, new Rotation2d(Math.toRadians(0))),
    FROM_STATION(CanonicalPose.STATION, new Rotation2d(Math.toRadians(0))),

    //top algae reef
    TO_A1(CanonicalPose.A1, new Rotation2d (Math.toRadians(95))), //goal pose is 95
    FROM_A1(CanonicalPose.A1, new Rotation2d(Math.toRadians(-85))),

    //bottom algae reef
    TO_A2(CanonicalPose.A2, new Rotation2d (Math.toRadians(95))), //goal pose is 95
    FROM_A2(CanonicalPose.A2, new Rotation2d(Math.toRadians(-85))),

    //home
    GOING_FORWARD(CanonicalPose.BETWEEN, Rotation2d.kCCW_90deg), //is there a going backward? what exatcly is this for?
    GOING_BACKWARD(CanonicalPose.BETWEEN, Rotation2d.kCW_90deg),

    //L4
    TO_L4(CanonicalPose.L4, new Rotation2d(Math.toRadians(125))),
    FROM_L4(CanonicalPose.L4, new Rotation2d(Math.toRadians(-50))), //just opposite of to

    //L3
    TO_L3(CanonicalPose.L3, new Rotation2d(Math.toRadians(120))),
    FROM_L3(CanonicalPose.L3, new Rotation2d(Math.toRadians(-50))),

    //L2
    TO_L2(CanonicalPose.L2, new Rotation2d(Math.toRadians(120))),
    FROM_L2(CanonicalPose.L2, new Rotation2d(Math.toRadians(-50))),

    //L1
    TO_L1(CanonicalPose.L1, new Rotation2d(Math.toRadians(100))), //goal rotaion for L1 SHould be 100 from vert
    FROM_L1(CanonicalPose.L1, new Rotation2d(Math.toRadians(-80))),

    //PROCESSOR
    TO_PROCESSOR(CanonicalPose.PROCESSOR, new Rotation2d(Math.toRadians(0))), //TODO: CALIRBARTE PROCESSOR TO FROM
    FROM_PROCESSOR(CanonicalPose.PROCESSOR, new Rotation2d(Math.toRadians(0))),

    //barge
    TO_BARGE(CanonicalPose.BARGE, new Rotation2d(Math.toRadians(113))), //as long as its down its good
    FROM_BARGE(CanonicalPose.BARGE, new Rotation2d(Math.toRadians(-67)));






    public final HolonomicPose2d hPose;

    private Waypoint(CanonicalPose pose, Rotation2d course) {
        hPose = new HolonomicPose2d(
                pose.pose.getTranslation(),
                pose.pose.getRotation(),
                course);
    }

}
