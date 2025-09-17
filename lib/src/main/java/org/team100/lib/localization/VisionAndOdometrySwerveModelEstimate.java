package org.team100.lib.localization;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.SideEffect;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

/**
 * Pull the estimate from the history after making sure that the history is up
 * to date.
 */
public class VisionAndOdometrySwerveModelEstimate implements SwerveModelEstimate {

    private final LimitedInterpolatingSwerveModelHistory m_history;
    private final SideEffect m_vision;
    private final SideEffect m_odometry;

    public VisionAndOdometrySwerveModelEstimate(
            AprilTagRobotLocalizer vision,
            OdometryUpdater odometry,
            LimitedInterpolatingSwerveModelHistory history) {
        m_history = history;
        m_vision = Cache.ofSideEffect(vision::update);
        m_odometry = Cache.ofSideEffect(odometry::update);
    }

    @Override
    public SwerveModel apply(double timestampS) {
        // run our dependencies if they haven't already
        m_vision.run();
        m_odometry.run();
        // query the history
        return m_history.apply(timestampS);
    }

}
