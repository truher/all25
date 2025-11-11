package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.tuning.Mutable;

/**
 * Linear velocity limit based on spatial yaw rate, drivetrain omega limit
 * (scaled), and drivetrain alpha limit (scaled).
 * 
 * Slows the path velocity to accommodate the desired yaw rate.
 * 
 * Does not affect maximum acceleration.
 */
public class YawRateConstraint implements TimingConstraint {
    private final Mutable m_maxOmegaRad_S;
    private final Mutable m_maxAlphaRad_S2;

    public YawRateConstraint(LoggerFactory parent, double maxOmega, double maxAlpha) {
        LoggerFactory log = parent.type(this);
        m_maxOmegaRad_S = new Mutable(log, "maxOmega", maxOmega);
        m_maxAlphaRad_S2 = new Mutable(log, "maxAlpha", maxAlpha);
    }

    /**
     * Use the factory.
     * 
     * @param limits absolute maxima
     * @param scale  apply to the maximum angular speed to get the actual
     *               constraint. The absolute maximum yaw rate is *very* high, and
     *               never useful for trajectories. A good number to try here might
     *               be 0.2.
     */
    public YawRateConstraint(LoggerFactory log, SwerveKinodynamics limits, double scale) {
        this(log, limits.getMaxAngleSpeedRad_S() * scale, limits.getMaxAngleAccelRad_S2() * scale);
    }

    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {       
        // Heading rate in rad/m
        final double heading_rate = state.getHeadingRateRad_M();
        // rad/s / rad/m => m/s.
        return new NonNegativeDouble(m_maxOmegaRad_S.getAsDouble() / Math.abs(heading_rate));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocity) {
        // Heading rate in rad/m
        final double heading_rate = state.getHeadingRateRad_M();
        // rad/s^2 / rad/m => m/s^2
        double limitM_S = m_maxAlphaRad_S2.getAsDouble() / Math.abs(heading_rate);
        return new MinMaxAcceleration(-limitM_S, limitM_S);
    }
}