package org.team100.lib.motion.servo;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

import edu.wpi.first.math.MathUtil;

/** Common elements of angular position servos. */
public abstract class AngularPositionServoImpl implements AngularPositionServo {
    protected static final double POSITION_TOLERANCE = 0.02;
    protected static final double VELOCITY_TOLERANCE = 0.02;
    protected final RotaryMechanism m_mechanism;
    protected final ProfileReference1d m_ref;
    private final DoubleLogger m_log_goal;

    /**
     * Goal is "unwrapped" i.e. it's it's [-inf, inf], not [-pi,pi]
     */
    Model100 m_unwrappedGoal = new Model100(0, 0);
    /**
     * Setpoint is "unwrapped" i.e. it's [-inf, inf], not [-pi,pi]
     */
    Control100 m_unwrappedSetpoint = new Control100(0, 0);

    protected AngularPositionServoImpl(LoggerFactory parent, RotaryMechanism mechanism, ProfileReference1d ref) {
        m_mechanism = mechanism;
        m_ref = ref;
        LoggerFactory child = parent.type(this);
        m_log_goal = child.doubleLogger(Level.TRACE, "goal (rad)");

    }

    abstract void actuate(Setpoints1d wrappedSetpoints, double torqueNm);

    @Override
    public void reset() {
        // using the current velocity sometimes includes a whole lot of noise, and then
        // the profile tries to follow that noise. so instead, use zero.
        Control100 measurement = new Control100(getWrappedPositionRad(), 0);
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
    public void setPositionDirect(Setpoints1d wrappedSetpoint, double torqueNm) {
        m_unwrappedGoal = null;
        actuate(wrappedSetpoint, torqueNm);
    }

    @Override
    public void setPositionProfiled(double wrappedGoalRad, double torqueNm) {
        m_log_goal.log(() -> wrappedGoalRad);

        // since the measurement is unwrapped, this yields the unwrapped goal.
        Model100 unwrappedGoal = new Model100(wrapNearMeasurement(wrappedGoalRad), 0);

        if (!unwrappedGoal.near(m_unwrappedGoal, POSITION_TOLERANCE, VELOCITY_TOLERANCE)) {
            m_unwrappedGoal = unwrappedGoal;
            m_ref.setGoal(unwrappedGoal);
            // make sure the setpoint is near the measurement
            if (m_unwrappedSetpoint == null) {
                // erased by dutycycle control
                m_unwrappedSetpoint = new Control100(m_mechanism.getWrappedPositionRad(), 0);
            } else {
                m_unwrappedSetpoint = new Control100(wrapNearMeasurement(m_unwrappedSetpoint.x()),
                        m_unwrappedSetpoint.v());
            }
            // initialize with the setpoint, not the measurement, to avoid noise.
            m_ref.init(m_unwrappedSetpoint.model());
        }

        actuate(m_ref.get(), torqueNm);
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        m_mechanism.setTorqueLimit(torqueNm);
    }

    /**
     * @return the absolute 1:1 position of the mechanism in [-pi, pi]
     */
    @Override
    public double getWrappedPositionRad() {
        return m_mechanism.getWrappedPositionRad();
    }

    @Override
    public double getUnwrappedPositionRad() {
        return m_mechanism.getUnwrappedPositionRad();
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

    /**
     * Given an position (either wrapped or unwrapped), return an equivalent
     * unwrapped position within pi of the unwrapped measurement.
     */
    double wrapNearMeasurement(double unwrappedPositionRad) {
        double unwrappedMeasurement = m_mechanism.getUnwrappedPositionRad();
        return MathUtil.angleModulus(unwrappedPositionRad - unwrappedMeasurement) + unwrappedMeasurement;
    }

}
