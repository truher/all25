package org.team100.lib.controller.r3;

import org.team100.lib.geometry.GlobalDeltaR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.GlobalVelocityR3Logger;
import org.team100.lib.state.ControlR3;

/**
 * Velocity feedforward and proportional feedback on position and velocity.
 */
public class FullStateControllerR3 extends ControllerR3Base {

    private final GlobalVelocityR3Logger m_log_u_FF;
    private final GlobalVelocityR3Logger m_log_u_FB;
    private final GlobalVelocityR3Logger m_log_u_VFB;

    private final double m_kPCart;
    private final double m_kPTheta;
    private final double m_kPCartV;
    private final double m_kPThetaV;

    public FullStateControllerR3(
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

        m_log_u_FF = log.globalVelocityR3Logger(Level.TRACE, "feedforward");
        m_log_u_FB = log.globalVelocityR3Logger(Level.TRACE, "position feedback");
        m_log_u_VFB = log.globalVelocityR3Logger(Level.TRACE, "velocity feedback");

        m_kPCart = kPCart;
        m_kPTheta = kPTheta;
        m_kPCartV = kPCartV;
        m_kPThetaV = kPThetaV;
    }

    @Override
    public GlobalVelocityR3 calculate100(
            GlobalDeltaR3 positionError,
            GlobalVelocityR3 velocityError,
            ControlR3 nextReference) {
        GlobalVelocityR3 u_FF = feedforward(nextReference);
        GlobalVelocityR3 u_FB = fullFeedback(positionError, velocityError);
        return u_FF.plus(u_FB);
    }

    ///////////////////////////////////////////////
    //
    // package-private for testing

    GlobalVelocityR3 feedforward(ControlR3 nextReference) {
        m_log_u_FF.log(() -> nextReference.velocity());
        return nextReference.velocity();
    }

    GlobalVelocityR3 fullFeedback(GlobalDeltaR3 positionError, GlobalVelocityR3 velocityError) {
        GlobalVelocityR3 u_XFB = positionFeedback(positionError);
        GlobalVelocityR3 u_VFB = velocityFeedback(velocityError);
        return u_XFB.plus(u_VFB);
    }

    /**
     * Returns position feedback proportional to position error.
     */
    GlobalVelocityR3 positionFeedback(GlobalDeltaR3 positionError) {
        GlobalVelocityR3 u_FB = new GlobalVelocityR3(
                m_kPCart * positionError.getX(),
                m_kPCart * positionError.getY(),
                m_kPTheta * positionError.getRotation().getRadians());
        m_log_u_FB.log(() -> u_FB);
        return u_FB;
    }

    /**
     * Returns velocity feedback proportional to velocity error.
     */
    GlobalVelocityR3 velocityFeedback(GlobalVelocityR3 velocityError) {
        GlobalVelocityR3 u_VFB = new GlobalVelocityR3(
                m_kPCartV * velocityError.x(),
                m_kPCartV * velocityError.y(),
                m_kPThetaV * velocityError.theta());
        m_log_u_VFB.log(() -> u_VFB);
        return u_VFB;
    }

}
