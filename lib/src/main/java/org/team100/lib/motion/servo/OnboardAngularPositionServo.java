package org.team100.lib.motion.servo;

import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

/**
 * Uses mechanism velocity control.
 * 
 * Uses a profile with velocity feedforward, feedback here in Java-land, and
 * extra torque (e.g. for gravity).
 */
public class OnboardAngularPositionServo extends AngularPositionServoImpl {
    private static final boolean DEBUG = false;

    private final Feedback100 m_feedback;

    private final DoubleLogger m_log_feedforward_torque;
    private final Model100Logger m_log_measurement;
    private final Control100Logger m_log_control;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_velocity_error;

    public OnboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            ProfileReference1d ref,
            Feedback100 feedback) {
        super(parent, mech, ref);
        LoggerFactory child = parent.type(this);
        m_feedback = feedback;

        m_log_feedforward_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = child.model100Logger(Level.COMP, "measurement (rad)");
        m_log_control = child.control100Logger(Level.COMP, "control (rad)");
        m_log_u_FB = child.doubleLogger(Level.TRACE, "u_FB (rad_s)");
        m_log_u_FF = child.doubleLogger(Level.TRACE, "u_FF (rad_s)");

        m_log_u_TOTAL = child.doubleLogger(Level.COMP, "u_TOTAL (rad_s)");
        m_log_error = child.doubleLogger(Level.TRACE, "Controller Position Error (rad)");
        m_log_velocity_error = child.doubleLogger(Level.TRACE, "Controller Velocity Error (rad_s)");
    }

    @Override
    public void reset() {
        super.reset();
        m_feedback.reset();
    }


    void actuate(Setpoints1d wrappedSetpoints, double feedForwardTorqueNm) {
        if (DEBUG)
            Util.printf("wrappedSetpoints %s\n", wrappedSetpoints);
        Control100 nextWrappedSetpoint = wrappedSetpoints.next();

        double nextPosRad = wrapNearMeasurement(nextWrappedSetpoint.x());
        double nextVelRad_S = nextWrappedSetpoint.v();
        double nextAccelRad_S2 = nextWrappedSetpoint.a();

        m_unwrappedSetpoint = new Control100(nextPosRad, nextVelRad_S, nextAccelRad_S2);

        final double unwrappedPositionRad = m_mechanism.getUnwrappedPositionRad();
        final double velocityRad_S = m_mechanism.getVelocityRad_S();

        final Model100 unwrappedMeasurement = new Model100(unwrappedPositionRad, velocityRad_S);

        double currentUnwrappedSetpointPosition = wrapNearMeasurement(wrappedSetpoints.current().x());
        double currentSetpointVelocity = wrappedSetpoints.current().v();

        Model100 currentUnwrappedSetpoint = new Model100(
                currentUnwrappedSetpointPosition, currentSetpointVelocity);

        final double u_FB = m_feedback.calculate(unwrappedMeasurement, currentUnwrappedSetpoint);

        final double u_FF = m_unwrappedSetpoint.v();
        final double u_TOTAL = u_FB + u_FF;

        m_mechanism.setVelocity(u_TOTAL, m_unwrappedSetpoint.a(), feedForwardTorqueNm);

        m_log_feedforward_torque.log(() -> feedForwardTorqueNm);
        m_log_measurement.log(() -> unwrappedMeasurement);
        m_log_control.log(() -> m_unwrappedSetpoint);
        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(() -> wrappedSetpoints.current().x() - unwrappedPositionRad);
        m_log_velocity_error.log(() -> wrappedSetpoints.current().v() - velocityRad_S);
    }


}
