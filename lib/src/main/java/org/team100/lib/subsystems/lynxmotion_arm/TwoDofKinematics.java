package org.team100.lib.subsystems.lynxmotion_arm;

import edu.wpi.first.math.geometry.Translation2d;

public interface TwoDofKinematics {

    /** Workspace end location based on joint configuration. */
    TwoDofArmPosition forward(TwoDofArmConfig a);

    /**
     * Calculate joint configuration given the workspace location of the end.
     * 
     * It's an application of the law of cosines.
     * 
     * Refer to the diagram:
     * https://docs.google.com/document/d/1B6vGPtBtnDSOpfzwHBflI8-nn98W9QvmrX78bon8Ajw
     */
    TwoDofArmConfig inverse(Translation2d t);

}