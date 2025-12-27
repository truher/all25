package org.team100.lib.controller.se2;

import org.team100.lib.geometry.DeltaSE2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.state.ControlSE2;

/**
 * A controller that doesn't do anything except calculate the errors, so that
 * "atReference" works.
 */
public class NullControllerSE2 extends ControllerSE2Base {

    public NullControllerSE2(
            LoggerFactory parent,
            double xTolerance,
            double thetaTolerance,
            double xDotTolerance,
            double omegaTolerance) {
        super(parent, xTolerance, thetaTolerance, xDotTolerance, omegaTolerance);
    }

    @Override
    public VelocitySE2 calculate100(
            DeltaSE2 positionError,
            VelocitySE2 velocityError,
            ControlSE2 nextReference) {
        return VelocitySE2.ZERO;
    }

}
