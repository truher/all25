package org.team100.lib.localization;

import java.util.Optional;

import org.team100.lib.motion.drivetrain.state.SwerveModel;

/**
 * Pull the estimate from the history after making sure that the history is up
 * to date.
 */
public class SwerveModelEstimate implements SwerveModelEstimateInterface {

    private final SwerveModelHistory m_history;

    public SwerveModelEstimate(SwerveModelHistory history) {
        m_history = history;
    }

    @Override
    public Optional<SwerveModel> get(double timestampS) {
        return m_history.get(timestampS);
    }

}
