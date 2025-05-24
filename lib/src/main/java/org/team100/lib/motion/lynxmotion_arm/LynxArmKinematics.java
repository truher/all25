package org.team100.lib.motion.lynxmotion_arm;

import edu.wpi.first.math.geometry.Pose3d;

public interface LynxArmKinematics {

    /**
     * Applies the configuration transforms in order to obtain the end-effector
     * (grip center) pose relative to the arm origin, which is the on the tabletop
     * at the swing axis.
     */
    LynxArmPose forward(LynxArmConfig joints);

    /**
     * Solve the inverse kinematics for the given end-effector pose.
     * 
     * Refer to the diagram
     * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
     */
    LynxArmConfig inverse(Pose3d end);

}