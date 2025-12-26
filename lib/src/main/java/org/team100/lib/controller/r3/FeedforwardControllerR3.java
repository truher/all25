package org.team100.lib.controller.r3;

import org.team100.lib.geometry.DeltaSE2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.VelocitySE2Logger;
import org.team100.lib.state.ControlR3;

/**
 * A controller that doesn't do anything except return the "next" setpoint. This
 * is appropriate if feedback control is outboard.
 */
public class FeedforwardControllerR3 extends ControllerR3Base {
    private final VelocitySE2Logger m_log_u_FF;

    public FeedforwardControllerR3(
            LoggerFactory parent,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        super(parent, xTolerance, thetaTolerance, xDotTolerance, omegaTolerance);
        LoggerFactory log = parent.type(this);
        m_log_u_FF = log.VelocitySE2Logger(Level.TRACE, "feedforward");

    }

    @Override
    public VelocitySE2 calculate100(
            DeltaSE2 positionError,
            VelocitySE2 velocityError,
            ControlR3 nextReference) {
        m_log_u_FF.log(() -> nextReference.velocity());
        return nextReference.velocity();
    }
}
