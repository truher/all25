package org.team100.lib.controller.r3;

import org.team100.lib.geometry.GlobalDeltaR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.ControlR3Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.GlobaDeltaR3Logger;
import org.team100.lib.logging.LoggerFactory.GlobalVelocityR3Logger;
import org.team100.lib.logging.LoggerFactory.ModelR3Logger;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Base class for R3 controllers.
 * 
 * Implements error calculation and tolerances.
 */
public abstract class ControllerR3Base implements ControllerR3 {

    private final ModelR3Logger m_log_measurement;
    private final ModelR3Logger m_log_currentReference;
    private final ControlR3Logger m_log_nextReference;

    private final GlobaDeltaR3Logger m_log_position_error;
    private final GlobalVelocityR3Logger m_log_velocity_error;
    /** Error in cartesian distance, i.e. hypot(x, y). */
    private final DoubleLogger m_log_cartesianPositionError;
    /** Error in cartesian velocity, i.e. hypot(vx, vy). */
    private final DoubleLogger m_log_cartesianVelocityError;

    private final BooleanLogger m_log_atPositionReference;
    private final BooleanLogger m_log_atVelocityReference;
    private final BooleanLogger m_log_atReference;

    private final double m_xTolerance;
    private final double m_thetaTolerance;
    private final double m_xDotTolerance;
    private final double m_omegaTolerance;

    /** The position error calculated in the most-recent call to calculate. */
    private GlobalDeltaR3 m_positionError;
    /** The velocity error calculated in the most-recent call to calculate. */
    private GlobalVelocityR3 m_velocityError;

    public ControllerR3Base(
            LoggerFactory parent,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        LoggerFactory log = parent.type(this);

        m_log_measurement = log.modelR3Logger(Level.DEBUG, "measurement");
        m_log_currentReference = log.modelR3Logger(Level.DEBUG, "current reference");
        m_log_nextReference = log.controlR3Logger(Level.DEBUG, "next reference");

        m_log_position_error = log.globalDeltaR3Logger(Level.TRACE, "position error");
        m_log_velocity_error = log.globalVelocityR3Logger(Level.TRACE, "velocity error");
        m_log_cartesianPositionError = log.doubleLogger(Level.TRACE, "cartesian position error");
        m_log_cartesianVelocityError = log.doubleLogger(Level.TRACE, "cartesian velocity error ");

        m_log_atPositionReference = log.booleanLogger(Level.TRACE, "at position reference");
        m_log_atVelocityReference = log.booleanLogger(Level.TRACE, "at velocity reference");
        m_log_atReference = log.booleanLogger(Level.TRACE, "at reference");

        m_xTolerance = xTolerance;
        m_thetaTolerance = thetaTolerance;
        m_xDotTolerance = xDotTolerance;
        m_omegaTolerance = omegaTolerance;
    }

    @Override
    public GlobalVelocityR3 calculate(
            ModelR3 measurement,
            ModelR3 currentReference,
            ControlR3 nextReference) {
        m_log_measurement.log(() -> measurement);
        m_log_currentReference.log(() -> currentReference);
        m_log_nextReference.log(() -> nextReference);
        m_positionError = positionError(measurement, currentReference);
        m_velocityError = velocityError(measurement, currentReference);
        return calculate100(m_positionError, m_velocityError, nextReference);
    }

    public abstract GlobalVelocityR3 calculate100(
            GlobalDeltaR3 positionError,
            GlobalVelocityR3 velocityError,
            ControlR3 nextReference);

    /**
     * Uses the position and velocity errors from the previous call to calculate().
     */
    @Override
    public boolean atReference() {
        if (m_positionError == null || m_velocityError == null)
            return false;
        boolean atReference1 = positionOK(m_positionError) && velocityOK(m_velocityError);
        m_log_atReference.log(() -> atReference1);
        return atReference1;
    }

    /** True if cartesian and rotation position errors are within tolerance. */
    boolean positionOK(GlobalDeltaR3 positionError) {
        double cartesian = positionError.getTranslation().getNorm();
        m_log_cartesianPositionError.log(() -> cartesian);
        double rotation = Math.abs(positionError.getRotation().getRadians());
        boolean withinTolerance = cartesian < m_xTolerance && rotation < m_thetaTolerance;
        m_log_atPositionReference.log(() -> withinTolerance);
        return withinTolerance;
    }

    /** True if cartesian and rotation velocity errors are within tolerance. */
    boolean velocityOK(GlobalVelocityR3 velocityError) {
        double cartesian = velocityError.norm();
        m_log_cartesianVelocityError.log(() -> cartesian);
        double rotation = Math.abs(velocityError.angle().orElse(Rotation2d.kZero).getRadians());
        boolean withinTolerance = cartesian < m_xDotTolerance && rotation < m_omegaTolerance;
        m_log_atVelocityReference.log(() -> withinTolerance);
        return withinTolerance;
    }

    /**
     * Wraps heading.
     */
    GlobalDeltaR3 positionError(ModelR3 measurement, ModelR3 currentReference) {
        GlobalDeltaR3 err = GlobalDeltaR3.delta(measurement.pose(), currentReference.pose());
        m_log_position_error.log(() -> err);
        return err;
    }

    /**
     * Velocity does not wrap.
     */
    GlobalVelocityR3 velocityError(ModelR3 measurement, ModelR3 currentReference) {
        GlobalVelocityR3 err = currentReference.velocity().minus(measurement.velocity());
        m_log_velocity_error.log(() -> err);
        return err;
    }

}
