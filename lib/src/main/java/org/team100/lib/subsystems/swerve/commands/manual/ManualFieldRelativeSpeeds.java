package org.team100.lib.subsystems.swerve.commands.manual;

import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.VelocitySE2Logger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;

/**
 * Transform manual input into a field-relative velocity.
 * 
 * The input is a twist, so the output is just scaled.
 */
public class ManualFieldRelativeSpeeds implements FieldRelativeDriver {
    private final SwerveKinodynamics m_swerveKinodynamics;
    // LOGGERS
    private final VelocitySE2Logger m_log_scaled;

    public ManualFieldRelativeSpeeds(LoggerFactory parent, SwerveKinodynamics swerveKinodynamics) {
        LoggerFactory log = parent.type(this);
        m_log_scaled = log.VelocitySE2Logger(Level.TRACE, "scaled");
        m_swerveKinodynamics = swerveKinodynamics;
    }

    /**
     * Clips the input to the unit circle, scales to maximum (not simultaneously
     * feasible) speeds.
     */
    @Override
    public VelocitySE2 apply(ModelSE2 state, Velocity input) {
        // clip the input to the unit circle
        final Velocity clipped = input.clip(1.0);

        // scale to max in both translation and rotation
        final VelocitySE2 scaled = FieldRelativeDriver.scale(
                clipped,
                m_swerveKinodynamics.getMaxDriveVelocityM_S(),
                m_swerveKinodynamics.getMaxAngleSpeedRad_S());

        m_log_scaled.log(() -> scaled);
        return scaled;
    }

    @Override
    public void reset(ModelSE2 p) {
        //
    }
}
