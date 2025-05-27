package org.team100.lib.motion.lynxmotion_arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public interface LynxArmKinematics {
    static final boolean DEBUG = false;

    /**
     * Applies the configuration transforms in order to obtain the end-effector
     * (grip center) pose relative to the arm origin, which is the on the tabletop
     * at the swing axis.
     */
    LynxArmPose forward(LynxArmConfig joints);

    /**
     * Solve the inverse kinematics for the given end-effector pose.
     * 
     * The initial config is useful for the solver: usually the new position is not
     * too far from the old one.
     * 
     * You probably want to "fix" the argument to this function.
     * 
     * Refer to the diagram
     * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
     */
    LynxArmConfig inverse(LynxArmConfig initial, Pose3d end);

    /**
     * The Lynxmotion arm doesn't allow arbitrary end-effector poses. If you supply
     * an "impossible" pose to the inverse kinematics, then the solver will fail.
     * That might be ok -- the "best approach" might be good enough, but it takes a
     * long time for the solver to decide that it's impossible, and it's not
     * necessarily deterministic. Instead, this method constraints the end-effector
     * pose to be feasible.
     * 
     * TODO: add workspace envelope fixing
     * 
     */
    static Pose3d fix(Pose3d p) {
        Translation3d x = new Translation3d(1, 0, 0);
        Translation3d rotated = x.rotateBy(p.getRotation());
        // is the resulting vector pointing up or down?
        Translation2d trans2d = rotated.toTranslation2d();
        if (trans2d.getNorm() < 1e-3) {
            // it's vertical so the twist can be free
            return p;
        }
        // it's not vertical.
        // The easiest way to "fix" it is to use the translation's yaw.
        double yaw = p.getTranslation().toTranslation2d().getAngle().getRadians();
        if (DEBUG)
            System.out.printf("yaw %s\n", yaw);
        return new Pose3d(new Translation3d(p.getX(), p.getY(), p.getZ()),
                new Rotation3d(p.getRotation().getX(), p.getRotation().getY(), yaw));
    }

}