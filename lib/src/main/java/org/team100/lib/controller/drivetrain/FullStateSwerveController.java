package org.team100.lib.controller.drivetrain;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeDeltaLogger;
import org.team100.lib.logging.LoggerFactory.FieldRelativeVelocityLogger;
import org.team100.lib.logging.LoggerFactory.SwerveControlLogger;
import org.team100.lib.logging.LoggerFactory.SwerveModelLogger;
import org.team100.lib.motion.drivetrain.state.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.state.GlobalSe2Velocity;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.util.Util;

/**
 * Velocity feedforward, proportional feedback on position and velocity.
 */
public class FullStateSwerveController implements SwerveController {
    private static final boolean DEBUG = false;
    private final SwerveModelLogger m_log_measurement;
    private final BooleanLogger m_log_atPositionReference;
    // private final BooleanLogger m_log_atVelocityReference;

    private final DoubleLogger m_log_PositionError;
    // private final DoubleLogger m_log_VelocityError;


    private final SwerveModelLogger m_log_currentReference;
    private final SwerveControlLogger m_log_nextReference;
    private final FieldRelativeVelocityLogger m_log_u_FF;
    private final FieldRelativeDeltaLogger m_log_position_error;
    private final FieldRelativeVelocityLogger m_log_u_FB;
    private final FieldRelativeVelocityLogger m_log_velocity_error;
    private final FieldRelativeVelocityLogger m_log_u_VFB;

    private final double m_kPCart;
    private final double m_kPTheta;
    private final double m_kPCartV;
    private final double m_kPThetaV;
    private final double m_xTolerance;
    private final double m_thetaTolerance;
    private final double m_xDotTolerance;
    // private final double m_omegaTolerance;

    private boolean m_atReference;

    public FullStateSwerveController(
            LoggerFactory parent,
            double kPCart,
            double kPTheta,
            double kPCartV,
            double kPThetaV,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        LoggerFactory log = parent.type(this);
        m_log_measurement = log.swerveModelLogger(Level.DEBUG, "measurement");
        m_log_currentReference = log.swerveModelLogger(Level.DEBUG, "currentReference");
        m_log_nextReference = log.swerveControlLogger(Level.DEBUG, "nextReference");
        m_log_u_FF = log.fieldRelativeVelocityLogger(Level.TRACE, "u_FF");
        m_log_position_error = log.fieldRelativeDeltaLogger(Level.TRACE, "positionError");
        m_log_u_FB = log.fieldRelativeVelocityLogger(Level.TRACE, "u_FB");
        m_log_velocity_error = log.fieldRelativeVelocityLogger(Level.TRACE, "velocityError");
        m_log_u_VFB = log.fieldRelativeVelocityLogger(Level.TRACE, "u_VFB");
        

        m_log_atPositionReference = log.booleanLogger(Level.TRACE, "At Position Reference");
        // m_log_atVelocityReference = log.booleanLogger(Level.TRACE, "At Velocity Reference");

        m_log_PositionError = log.doubleLogger(Level.TRACE, "Position Error ");
        // m_log_VelocityError = log.doubleLogger(Level.TRACE, "Velocity Error ");


        m_kPCart = kPCart;
        m_kPTheta = kPTheta;
        m_kPCartV = kPCartV;
        m_kPThetaV = kPThetaV;

        m_xTolerance = xTolerance;
        m_thetaTolerance = thetaTolerance;
        m_xDotTolerance = xDotTolerance;
        // m_omegaTolerance = omegaTolerance;
    }

    @Override
    public GlobalSe2Velocity calculate(
            SwerveModel measurement,
            SwerveModel currentReference,
            SwerveControl nextReference) {
        if (DEBUG)
            Util.printf("measurement %s current %s next %s\n",
                    measurement, currentReference, nextReference);
        m_log_measurement.log(() -> measurement);
        m_log_currentReference.log(() -> currentReference);
        m_log_nextReference.log(() -> nextReference);
        GlobalSe2Velocity u_FF = feedforward(nextReference);
        GlobalSe2Velocity u_FB = fullFeedback(measurement, currentReference);
        if (DEBUG)
            Util.printf("ff %s fb %s\n", u_FF, u_FB);
        return u_FF.plus(u_FB);
    }

    @Override
    public boolean atReference() {
        return m_atReference;
    }

    @Override
    public void reset() {
        //
    }

    ///////////////////////////////////////////////
    //
    // package-private for testing

    GlobalSe2Velocity feedforward(SwerveControl nextReference) {
        m_log_u_FF.log(() -> nextReference.velocity());
        return nextReference.velocity();
    }

    GlobalSe2Velocity fullFeedback(SwerveModel measurement, SwerveModel currentReference) {
        // atReference is updated below
        m_atReference = true;
        GlobalSe2Velocity u_XFB = positionFeedback(measurement, currentReference);
        GlobalSe2Velocity u_VFB = velocityFeedback(measurement, currentReference);
        return u_XFB.plus(u_VFB);
    }

    GlobalSe2Velocity positionFeedback(SwerveModel measurement, SwerveModel currentReference) {
        // wraps heading
        FieldRelativeDelta positionError = positionError(measurement, currentReference);
        m_atReference &= positionError.getTranslation().getNorm() < m_xTolerance
                && Math.abs(positionError.getRotation().getRadians()) < m_thetaTolerance;
        
        GlobalSe2Velocity u_FB = new GlobalSe2Velocity(
                m_kPCart * positionError.getX(),
                m_kPCart * positionError.getY(),
                m_kPTheta * positionError.getRotation().getRadians());

        m_log_atPositionReference.log( () -> positionError.getTranslation().getNorm() < m_xTolerance
        && Math.abs(positionError.getRotation().getRadians()) < m_thetaTolerance);
        // Util.println("THeta Tolerance" + (Math.abs(positionError.getRotation().getRadians()) < m_thetaTolerance));
        // Util.println("Position Tolerance" + (positionError.getTranslation().getNorm() < m_xTolerance));
        m_log_PositionError.log( () -> positionError.getTranslation().getNorm());
        m_log_u_FB.log(() -> u_FB);
        return u_FB;
    }

    GlobalSe2Velocity velocityFeedback(SwerveModel currentPose, SwerveModel currentReference) {
        final GlobalSe2Velocity velocityError = velocityError(currentPose, currentReference);
        m_atReference &= velocityError.norm() < m_xDotTolerance;
                // && Math.abs(velocityError.angle().orElse(Rotation2d.kZero).getRadians()) < m_omegaTolerance;

        final GlobalSe2Velocity u_VFB = new GlobalSe2Velocity(
                m_kPCartV * velocityError.x(),
                m_kPCartV * velocityError.y(),
                m_kPThetaV * velocityError.theta());
        m_log_u_VFB.log(() -> u_VFB);
        // Util.println("Omega Tolerance" + (Math.abs(velocityError.angle().orElse(Rotation2d.kZero).getRadians()) < m_omegaTolerance));

        // Util.println("Velocity Tolerance" + (velocityError.norm() < m_xTolerance));

        return u_VFB;
    }

    FieldRelativeDelta positionError(SwerveModel measurement, SwerveModel currentReference) {
        FieldRelativeDelta positionError = FieldRelativeDelta.delta(measurement.pose(), currentReference.pose());
        m_log_position_error.log(() -> positionError);
        return positionError;
    }

    GlobalSe2Velocity velocityError(SwerveModel measurement, SwerveModel currentReference) {
        GlobalSe2Velocity velocityError = currentReference.minus(measurement).velocity();
        m_log_velocity_error.log(() -> velocityError);
        return velocityError;
    }

}
