package org.team100.lib.controller.se2;

import org.team100.lib.geometry.DeltaSE2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.VelocitySE2Logger;
import org.team100.lib.state.ControlSE2;

/**
 * Velocity feedforward and proportional feedback on position and velocity.
 */
public class FullStateControllerSE2 extends ControllerSE2Base {

    private final VelocitySE2Logger m_log_u_FF;
    private final VelocitySE2Logger m_log_u_FB;
    private final VelocitySE2Logger m_log_u_VFB;

    private final double m_kPCart;
    private final double m_kPTheta;
    private final double m_kPCartV;
    private final double m_kPThetaV;

    public FullStateControllerSE2(
            LoggerFactory parent,
            double kPCart,
            double kPTheta,
            double kPCartV,
            double kPThetaV,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        super(parent, xTolerance, thetaTolerance, xDotTolerance, omegaTolerance);
        LoggerFactory log = parent.type(this);

        m_log_u_FF = log.VelocitySE2Logger(Level.TRACE, "feedforward");
        m_log_u_FB = log.VelocitySE2Logger(Level.TRACE, "position feedback");
        m_log_u_VFB = log.VelocitySE2Logger(Level.TRACE, "velocity feedback");

        m_kPCart = kPCart;
        m_kPTheta = kPTheta;
        m_kPCartV = kPCartV;
        m_kPThetaV = kPThetaV;
    }

    @Override
    public VelocitySE2 calculate100(
            DeltaSE2 positionError,
            VelocitySE2 velocityError,
            ControlSE2 nextReference) {
        VelocitySE2 u_FF = feedforward(nextReference);
        VelocitySE2 u_FB = fullFeedback(positionError, velocityError);
        return u_FF.plus(u_FB);
    }

    ///////////////////////////////////////////////
    //
    // package-private for testing

    VelocitySE2 feedforward(ControlSE2 nextReference) {
        m_log_u_FF.log(() -> nextReference.velocity());
        return nextReference.velocity();
    }

    VelocitySE2 fullFeedback(DeltaSE2 positionError, VelocitySE2 velocityError) {
        VelocitySE2 u_XFB = positionFeedback(positionError);
        VelocitySE2 u_VFB = velocityFeedback(velocityError);
        return u_XFB.plus(u_VFB);
    }

    /**
     * Returns position feedback proportional to position error.
     */
    VelocitySE2 positionFeedback(DeltaSE2 positionError) {
        VelocitySE2 u_FB = new VelocitySE2(
                m_kPCart * positionError.getX(),
                m_kPCart * positionError.getY(),
                m_kPTheta * positionError.getRotation().getRadians());
        m_log_u_FB.log(() -> u_FB);
        return u_FB;
    }

    /**
     * Returns velocity feedback proportional to velocity error.
     */
    VelocitySE2 velocityFeedback(VelocitySE2 velocityError) {
        VelocitySE2 u_VFB = new VelocitySE2(
                m_kPCartV * velocityError.x(),
                m_kPCartV * velocityError.y(),
                m_kPThetaV * velocityError.theta());
        m_log_u_VFB.log(() -> u_VFB);
        return u_VFB;
    }

}
