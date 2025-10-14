package org.team100.lib.motion.servo;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;

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

        Control100 nextUnwrappedSetpoint = positionNearMeasurement(
                wrappedSetpoint.next());

        m_nextUnwrappedSetpoint = nextUnwrappedSetpoint;

        m_mechanism.setUnwrappedPosition(
                m_nextUnwrappedSetpoint.x(),
                m_nextUnwrappedSetpoint.v(),
                m_nextUnwrappedSetpoint.a(),
                torqueNm);
        m_log_control.log(() -> m_nextUnwrappedSetpoint);
        m_log_ff_torque.log(() -> torqueNm);
    }

}
