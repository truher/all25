package org.team100.lib.motion.servo;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.util.Util;

/**
 * Uses mechanism position control.
 * 
 * Uses a profile with velocity feedforward, also extra torque (e.g. for
 * gravity). There's no feedback at this level, and no feedforward calculation
 * either: the mechanism does that.
 * 
 * Must be used with a combined encoder, to "zero" the motor encoder so that
 * positional commands make sense.
 */
public class OutboardAngularPositionServo extends AngularPositionServoImpl {
    private static final boolean DEBUG = false;

    private final DoubleLogger m_log_ff_torque;
    private final Control100Logger m_log_control;

    public OutboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            ProfileReference1d ref) {
        super(parent, mech, ref);
        LoggerFactory child = parent.type(this);
        m_log_ff_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_control = child.control100Logger(Level.TRACE, "setpoint (rad)");
    }

    /**
     * Pass the setpoint directly to the mechanism's position controller.
     * Ignores current setpoint. We only use the "next" setpoint.
     */
    void actuate(Setpoints1d wrappedSetpoint, double torqueNm) {
        Control100 nextWrappedSetpoint = wrappedSetpoint.next();
        if (DEBUG)
            Util.printf("next wrapped setpoint %6.3f\n", nextWrappedSetpoint.x());

        double positionRad = wrapNearMeasurement(nextWrappedSetpoint.x());
        double velocityRad_S = nextWrappedSetpoint.v();
        double accelRad_S2 = nextWrappedSetpoint.a();

        m_unwrappedSetpoint = new Control100(positionRad, velocityRad_S, accelRad_S2);

        if (DEBUG)
            Util.printf("position %6.3f\n", positionRad);

        m_mechanism.setUnwrappedPosition(
                positionRad,
                velocityRad_S,
                accelRad_S2,
                torqueNm);
        m_log_control.log(() -> m_unwrappedSetpoint);
        m_log_ff_torque.log(() -> torqueNm);
    }

}
