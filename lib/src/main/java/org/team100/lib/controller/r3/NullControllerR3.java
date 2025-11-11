package org.team100.lib.controller.r3;

import org.team100.lib.geometry.GlobalDeltaR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.state.ControlR3;

/**
 * A controller that doesn't do anything except calculate the errors, so that
 * "atReference" works.
 */
public class NullControllerR3 extends ControllerR3Base {

    public NullControllerR3(
            LoggerFactory parent,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        super(parent, xTolerance, thetaTolerance, xDotTolerance, omegaTolerance);
    }

    @Override
    public GlobalVelocityR3 calculate100(
            GlobalDeltaR3 positionError,
            GlobalVelocityR3 velocityError,
            ControlR3 nextReference) {
        return GlobalVelocityR3.ZERO;
    }

}
