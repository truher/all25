package org.team100.frc2025.CalgamesArm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * We often use an enum of canonical poses.
 */
public enum CanonicalPose {
    //wherever positions
    // WHEREVER

    HOME(1, 0, 0), //the fully hidden position
    PICK(0.1, -0.6, Rotation2d.kCW_90deg.getRadians()),
    /** This is required to make rotation go the right way around */
    BETWEEN(1, 0, 0), //between scoring posiition, needs to be caliprated
    L4(1.9, 0.5, Math.toRadians(150)), //make standard on where tool point is (5cm from where) also what does the shoulder look like (horiz?)
    L3(1.2, 0.5, Math.toRadians(120)), //angles are also wrong, measured from strage up zero. (Note: there is a disconnect between the angle of manipulator and the coral itself)
    L2(0.85, 0.5, Math.toRadians(120)),
    L1(0.5, 0.5, Math.toRadians(90)),
    STATION(0,0, Math.toRadians(2)),
    APICK(0,0, Math.toRadians(0)),
    A1(0,0,Math.toRadians(0)),
    A2(0,0,Math.toRadians(0)),
    BARGE(0,0,Math.toRadians(0)),
    PROCESSOR(0,0,Math.toRadians(0)),
    CLIMB(0,0,Math.toRadians(0)); //climb position, needs to be calibrated

    //should add a station pickup
    //add algae reef pickup
    //add alage ground pickup
    //add barge scoring
    //algae has differnet angle cuz shit

    public final Pose2d pose;

    private CanonicalPose(double x, double y, double r) {
        pose = new Pose2d(x, y, new Rotation2d(r));
    }

}
