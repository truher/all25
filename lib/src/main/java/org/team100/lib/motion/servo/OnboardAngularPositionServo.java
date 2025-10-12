package org.team100.lib.motion.servo;

import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.logging.LoggerFactory.Model100Logger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

/**
 * Uses mechanism velocity control.
 * 
 * Uses a profile with velocity feedforward, feedback here in Java-land, and
 * extra torque (e.g. for gravity).
 */
public class OnboardAngularPositionServo implements AngularPositionServo {
    private static final boolean DEBUG = false;
    private static final double POSITION_TOLERANCE = 0.02;
    private static final double VELOCITY_TOLERANCE = 0.02;

    private final RotaryMechanism m_mechanism;
    private final ProfileReference1d m_ref;
    private final Feedback100 m_feedback;

    private final DoubleLogger m_log_goal;
    private final DoubleLogger m_log_feedforward_torque;
    private final Model100Logger m_log_measurement;
    private final Control100Logger m_log_control;
    private final DoubleLogger m_log_u_FB;
    private final DoubleLogger m_log_u_FF;
    private final DoubleLogger m_log_u_TOTAL;
    private final DoubleLogger m_log_error;
    private final DoubleLogger m_log_velocity_error;
    private final DoubleLogger m_encoderValue;
    private final BooleanLogger m_log_at_setpoint;

    /**
     * Goal is "unwrapped" i.e. it's it's [-inf, inf], not [-pi,pi]
     */
    private Model100 m_unwrappedGoal;
    /**
     * Most-recent "next" setpoint
     * 
     * Setpoint is "unwrapped" i.e. it's [-inf, inf], not [-pi,pi]
     */
    Control100 m_unwrappedSetpoint;

    public OnboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            ProfileReference1d ref,
            Feedback100 feedback) {
        LoggerFactory child = parent.type(this);
        m_mechanism = mech;
        m_ref = ref;
        m_feedback = feedback;

        m_log_goal = child.doubleLogger(Level.COMP, "goal (rad)");
        m_log_feedforward_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = child.model100Logger(Level.COMP, "measurement (rad)");
        m_log_control = child.control100Logger(Level.COMP, "control (rad)");
        m_log_u_FB = child.doubleLogger(Level.TRACE, "u_FB (rad_s)");
        m_log_u_FF = child.doubleLogger(Level.TRACE, "u_FF (rad_s)");
        m_encoderValue = child.doubleLogger(Level.TRACE, "Encoder Value");

        m_log_u_TOTAL = child.doubleLogger(Level.COMP, "u_TOTAL (rad_s)");
        m_log_error = child.doubleLogger(Level.TRACE, "Controller Position Error (rad)");
        m_log_velocity_error = child.doubleLogger(Level.TRACE, "Controller Velocity Error (rad_s)");
        m_log_at_setpoint = child.booleanLogger(Level.TRACE, "At Setpoint");
    }

    /**
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    @Override
    public void reset() {
        if (DEBUG) {
            Util.println("OnboardAngularPositionServo reset");
        }
        // using the current velocity sometimes includes a whole lot of noise, and then
        // the profile tries to follow that noise. so instead, use zero.
        Control100 measurement = new Control100(getWrappedPositionRad(), 0);
        m_unwrappedSetpoint = measurement;
        // measurement is used for initialization only here.
        m_ref.setGoal(measurement.model());
        m_ref.init(measurement.model());
        m_feedback.reset();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        m_unwrappedGoal = null;
        m_unwrappedSetpoint = null;
        m_mechanism.setDutyCycle(dutyCycle);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_mechanism.setTorqueLimit(torqueNm);
    }

    @Override
    public void setPositionProfiled(double goalRad, double torqueNm) {
        m_log_goal.log(() -> goalRad);

        Model100 goal = new Model100(goalRad, 0);
        if (!goal.near(m_unwrappedGoal, POSITION_TOLERANCE, VELOCITY_TOLERANCE)) {
            m_unwrappedGoal = goal;
            m_ref.setGoal(goal);
            if (m_unwrappedSetpoint == null) {
                double measurement = m_mechanism.getWrappedPositionRad();
                // avoid velocity noise here
                m_unwrappedSetpoint = new Control100(measurement, 0);
            } else {
                m_unwrappedSetpoint = new Control100(mod(m_unwrappedSetpoint.x()), m_unwrappedSetpoint.v());
            }
            m_ref.init(m_unwrappedSetpoint.model());

        }
        actuate(m_ref.get(), torqueNm);
    }

    /**
     * set mechanism velocity use the current setpoint for feedback, and
     * the next setpoint velocity as feedforward.
     */
    @Override
    public void setPositionDirect(Setpoints1d setpoint, double torqueNm) {
        m_unwrappedGoal = null;
        actuate(setpoint, torqueNm);
    }

    private void actuate(Setpoints1d setpoints, double feedForwardTorqueNm) {
        m_unwrappedSetpoint = setpoints.next();

        final double positionRad = m_mechanism.getWrappedPositionRad();
        final double velocityRad_S = m_mechanism.getVelocityRad_S();

        final Model100 measurement = new Model100(positionRad, velocityRad_S);

        double setpointPosition = mod(setpoints.current().x());
        double setpointVelocity = setpoints.current().v();
        final double u_FB = m_feedback.calculate(
                measurement,
                new Model100(setpointPosition, setpointVelocity));
        final double u_FF = m_unwrappedSetpoint.v();
        final double u_TOTAL = u_FB + u_FF;

        m_mechanism.setVelocity(u_TOTAL, m_unwrappedSetpoint.a(), feedForwardTorqueNm);

        m_log_feedforward_torque.log(() -> feedForwardTorqueNm);
        m_log_measurement.log(() -> measurement);
        m_log_control.log(() -> m_unwrappedSetpoint);
        m_log_u_FB.log(() -> u_FB);
        m_log_u_FF.log(() -> u_FF);
        m_log_u_TOTAL.log(() -> u_TOTAL);
        m_log_error.log(() -> setpoints.current().x() - positionRad);
        m_log_velocity_error.log(() -> setpoints.current().v() - velocityRad_S);
    }

    /** Return an angle near the measurement */
    private double mod(double x) {
        double measurement = m_mechanism.getWrappedPositionRad();
        return MathUtil.angleModulus(x - measurement) + measurement;
    }

    /**
     * @return the absolute 1:1 position of the mechanism in [-pi, pi]
     */
    @Override
    public double getWrappedPositionRad() {
        return m_mechanism.getWrappedPositionRad();
    }

    @Override
    public boolean atSetpoint() {
        boolean atSetpoint = m_feedback.atSetpoint();
        m_log_at_setpoint.log(() -> atSetpoint);
        return atSetpoint;
    }

    @Override
    public boolean profileDone() {
        if (m_unwrappedGoal == null) {
            // if there's no profile, it's always done.
            return true;
        }
        return m_ref.profileDone();
    }

    @Override
    public boolean atGoal() {
        return atSetpoint() && profileDone();
    }

    @Override
    public void stop() {
        m_mechanism.stop();
    }

    @Override
    public void close() {
        m_mechanism.close();
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
        m_encoderValue.log(() -> getWrappedPositionRad());
    }
}
