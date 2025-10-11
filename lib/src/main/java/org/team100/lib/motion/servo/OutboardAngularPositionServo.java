package org.team100.lib.motion.servo;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Control100Logger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;

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
public class OutboardAngularPositionServo implements AngularPositionServo {
    private static final double POSITION_TOLERANCE = 0.05;
    private static final double VELOCITY_TOLERANCE = 0.05;

    private final RotaryMechanism m_mechanism;
    private final ProfileReference1d m_ref;

    private final DoubleLogger m_log_goal;
    private final DoubleLogger m_log_ff_torque;
    private final DoubleLogger m_log_measurement;
    private final Control100Logger m_log_setpoint;

    /**
     * Goal is "unwrapped" i.e. it's it's [-inf, inf], not [-pi,pi]
     */
    private Model100 m_unwrappedGoal = new Model100(0, 0);
    /**
     * Setpoint is "unwrapped" i.e. it's [-inf, inf], not [-pi,pi]
     */
    private Control100 m_unwrappedSetpoint = new Control100(0, 0);

    public OutboardAngularPositionServo(
            LoggerFactory parent,
            RotaryMechanism mech,
            ProfileReference1d ref) {
        LoggerFactory child = parent.type(this);
        m_mechanism = mech;
        m_ref = ref;
        m_log_goal = child.doubleLogger(Level.TRACE, "goal (rad)");
        m_log_ff_torque = child.doubleLogger(Level.TRACE, "Feedforward Torque (Nm)");
        m_log_measurement = child.doubleLogger(Level.TRACE, "measurement (rad)");
        m_log_setpoint = child.control100Logger(Level.TRACE, "setpoint (rad)");
    }

    @Override
    public void reset() {
        double position = getWrappedPositionRad();
        // using the current velocity sometimes includes a whole lot of noise, and then
        // the profile tries to follow that noise. so instead, use zero.
        Control100 measurement = new Control100(position, 0);
        m_unwrappedSetpoint = measurement;
        m_ref.setGoal(measurement.model());
        m_ref.init(measurement.model());
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
    public void setPositionProfiled(double goalRad, double feedForwardTorqueNm) {
        m_log_goal.log(() -> goalRad);

        Model100 goal = new Model100(mod(goalRad), 0);

        if (!goal.near(m_unwrappedGoal, POSITION_TOLERANCE, VELOCITY_TOLERANCE)) {
            m_unwrappedGoal = goal;
            m_ref.setGoal(goal);
            // make sure the setpoint is near the measurement
            if (m_unwrappedSetpoint == null) {
                // erased by dutycycle control
                m_unwrappedSetpoint = new Control100(m_mechanism.getWrappedPositionRad(), 0);
            } else {
                m_unwrappedSetpoint = new Control100(mod(m_unwrappedSetpoint.x()), m_unwrappedSetpoint.v());
            }
            // initialize with the setpoint, not the measurement, to avoid noise.
            m_ref.init(m_unwrappedSetpoint.model());
        }

        actuate(m_ref.get(), feedForwardTorqueNm);
    }

    /**
     * set mechanism position passthrough, adjusting the setpoint to be close to the
     * measurement.
     * invalidates the current profile.
     * for outboard control we only use the "next" setpoint.
     */
    @Override
    public void setPositionDirect(Setpoints1d setpoint, double feedForwardTorqueNm) {
        m_unwrappedGoal = null;
        actuate(setpoint, feedForwardTorqueNm);
    }

    /**
     * Pass the setpoint directly to the mechanism's position controller.
     * For outboard control we only use the "next" setpoint.
     */
    private void actuate(Setpoints1d setpoint, double torqueNm) {
        m_unwrappedSetpoint = setpoint.next();

        double positionRad = mod(m_unwrappedSetpoint.x());
        double velocityRad_S = m_unwrappedSetpoint.v();
        double accelRad_S2 = m_unwrappedSetpoint.a();

        m_mechanism.setUnwrappedPosition(
                positionRad,
                velocityRad_S,
                accelRad_S2,
                torqueNm);
        m_log_setpoint.log(() -> m_unwrappedSetpoint);
        m_log_ff_torque.log(() -> torqueNm);
    }

    /** Return an angle near the measurement */
    double mod(double x) {
        double measurement = m_mechanism.getWrappedPositionRad();
        m_log_measurement.log(() -> measurement);
        return MathUtil.angleModulus(x - measurement) + measurement;
    }

    /**
     * @return the absolute 1:1 position of the mechanism in [-pi, pi]
     */
    @Override
    public double getWrappedPositionRad() {
        return m_mechanism.getWrappedPositionRad();
    }

    /**
     * Compares robotPeriodic-updated measurements to the setpoint,
     * so you need to know when the setpoint was updated: is it for the
     * current Takt time, or the next step?
     */
    @Override
    public boolean atSetpoint() {
        double positionError = MathUtil.angleModulus(m_unwrappedSetpoint.x() - m_mechanism.getWrappedPositionRad());
        double velocityError = m_unwrappedSetpoint.v() - m_mechanism.getVelocityRad_S();
        return Math.abs(positionError) < POSITION_TOLERANCE
                && Math.abs(velocityError) < VELOCITY_TOLERANCE;
    }

    @Override
    public boolean profileDone() {
        if (m_unwrappedGoal == null) {
            // if there's no profile, it's always done.
            return true;
        }
        return m_ref.profileDone();
    }

    /**
     * Note this is affected by the setpoint update.
     * 
     * It really makes the most sense to call this *before* updating the setpoint,
     * because the measurement comes from the recent-past Takt and the updated
     * setpoint will be aiming at the next one.
     */
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
    }
}
