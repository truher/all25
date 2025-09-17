package org.team100.lib.localization;

import java.util.function.DoubleFunction;

import org.team100.lib.motion.drivetrain.state.SwerveModel;

/**
 * The SwerveModelEstimate is a proxy around the history that first makes sure
 * the history has received any updates that may mutate it. Clients should use
 * this interface to query the robot state, not the history directly.
 */
public interface SwerveModelEstimate extends DoubleFunction<SwerveModel> {

    /**
     * Provide the best estimate for SwerveModel at the given timestamp, first
     * making sure any pending updates from vision or odometry have been applied.
     */
    @Override
    SwerveModel apply(double timestampS);
}
