package org.team100.lib.localization;

import org.team100.lib.motion.drivetrain.state.SwerveModel;

/** For testing */
public interface PoseEstimator100 {

    SwerveModel get(double timestampS);
}
