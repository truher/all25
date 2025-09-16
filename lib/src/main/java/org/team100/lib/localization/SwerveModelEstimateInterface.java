package org.team100.lib.localization;

import java.util.Optional;

import org.team100.lib.motion.drivetrain.state.SwerveModel;

/** For testing */
public interface SwerveModelEstimateInterface {

    Optional<SwerveModel> get(double timestampS);
}
