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

/**
 * Common elements of angular position servos.
 * 
 * This uses the "short way" between measurement and goal or setpoint, unless
 * that path exceeds the mechanism bounds. In that case, it takes the "long
 * way".
 */
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
     * This is the setpoint for the "next" time step, i.e. the one we use for
     * feedforward.
     */
    Control100 m_nextUnwrappedSetpoint = new Control100(0, 0);

    /**
     * When the goal or setpoint is in an inaccessible zone, we hold position, so
     * there is a setpoint, but it's not the one the client asked for.
     */
    boolean m_validSetpoint;

    protected AngularPositionServoImpl(
            LoggerFactory parent,
            RotaryMechanism mechanism,
            ProfileReference1d ref) {
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
        m_nextUnwrappedSetpoint = measurement;
        m_ref.setGoal(measurement.model());
        m_ref.init(measurement.model());
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        m_unwrappedGoal = null;
        m_nextUnwrappedSetpoint = null;
        m_mechanism.setDutyCycle(dutyCycle);
    }

    Control100 positionNearMeasurement(Control100 c) {
        double x = nearMeasurement(c.x());
        double v = c.v();
        double a = c.a();
        return new Control100(x, v, a);
    }

    Model100 positionNearMeasurement(Model100 m) {
        double x = nearMeasurement(m.x());
        double v = m.v();
        return new Model100(x, v);
    }

    @Override
    public void setPositionDirect(Setpoints1d wrappedSetpoint, double torqueNm) {
        // make sure the reference gets reinitialized if required later
        m_unwrappedGoal = null;
        m_validSetpoint = true;

        double unwrappedMeasurement = m_mechanism.getUnwrappedPositionRad();
        double nextDx = MathUtil.angleModulus(wrappedSetpoint.next().x() - unwrappedMeasurement);
        double nextX = unwrappedMeasurement + nextDx;
        if (nextDx > 0) {
            // short way is positive
            if (nextX > m_mechanism.getMaxPositionRad()) {
                // short way is beyond the limit; go around
                nextX = nextX - 2 * Math.PI;
                if (nextX < m_mechanism.getMinPositionRad()) {
                    // setpoint is inaccessible, hold position.
                    m_nextUnwrappedSetpoint = m_mechanism.getUnwrappedMeasurement().control();
                    actuate(new Setpoints1d(m_nextUnwrappedSetpoint, m_nextUnwrappedSetpoint), torqueNm);
                    m_validSetpoint = false;
                    return;
                }
            }
        } else {
            // short way is negative
            if (nextX < m_mechanism.getMinPositionRad()) {
                // short way is beyond the limit; go around
                nextX = nextX + 2 * Math.PI;
                if (nextX > m_mechanism.getMaxPositionRad()) {
                    // setpoint is inaccessible; hold position.
                    m_nextUnwrappedSetpoint = m_mechanism.getUnwrappedMeasurement().control();
                    actuate(new Setpoints1d(m_nextUnwrappedSetpoint, m_nextUnwrappedSetpoint), torqueNm);
                    m_validSetpoint = false;
                    return;
                }
            }
        }
        m_nextUnwrappedSetpoint = new Control100(
                nextX,
                wrappedSetpoint.next().v(),
                wrappedSetpoint.next().a());
        double nextMinusCurrentX = MathUtil.angleModulus(
                wrappedSetpoint.next().x() - wrappedSetpoint.current().x());
        double curX = nextX - nextMinusCurrentX;
        Setpoints1d unwrappedSetpoint = new Setpoints1d(
                new Control100(curX, wrappedSetpoint.current().v(), wrappedSetpoint.current().a()),
                m_nextUnwrappedSetpoint);
        actuate(unwrappedSetpoint, torqueNm);
    }

    @Override
    public void setPositionProfiled(double wrappedGoalRad, double torqueNm) {
        m_log_goal.log(() -> wrappedGoalRad);
        m_validSetpoint = true;
        double unwrappedMeasurement = m_mechanism.getUnwrappedPositionRad();
        double goalDx = MathUtil.angleModulus(wrappedGoalRad - unwrappedMeasurement);
        double goalX = unwrappedMeasurement + goalDx;
        if (goalDx > 0) {
            // short way is positive
            if (goalX > m_mechanism.getMaxPositionRad()) {
                // goal is beyond the mechanism range, go the other way.
                goalX = goalX - 2 * Math.PI;
                if (goalX < m_mechanism.getMinPositionRad()) {
                    // the goal is inaccessible, just hold position.
                    goalX = unwrappedMeasurement;
                    m_validSetpoint = false;
                }
            }
        } else {
            // short way is negative
            if (goalX < m_mechanism.getMinPositionRad()) {
                // goal is too far, try an equivalent goal
                goalX = goalX + 2 * Math.PI;
                if (goalX > m_mechanism.getMaxPositionRad()) {
                    // the goal is inaccessible, just hold position.
                    goalX = unwrappedMeasurement;
                    m_validSetpoint = false;
                }
            }
        }
        initReference(new Model100(goalX, 0));
        Setpoints1d unwrappedSetpoint = m_ref.get();
        m_nextUnwrappedSetpoint = unwrappedSetpoint.next();
        actuate(unwrappedSetpoint, torqueNm);
    }

    /** The reference only understands unwrapped angles. */
    private void initReference(Model100 unwrappedGoal) {
        if (unwrappedGoal.near(m_unwrappedGoal, POSITION_TOLERANCE, VELOCITY_TOLERANCE)) {
            // If the new goal is the same as the old goal, no change is needed.
            return;
        }
        // The new goal is not the same as the old goal, so tell the reference about it.
        m_unwrappedGoal = unwrappedGoal;
        m_ref.setGoal(unwrappedGoal);
        // make sure the setpoint is near the measurement
        if (m_nextUnwrappedSetpoint == null) {
            // erased by dutycycle control, use measurement
            m_nextUnwrappedSetpoint = new Control100(m_mechanism.getUnwrappedPositionRad(), 0);
        }

        // initialize with the setpoint, not the measurement, to avoid noise.
        m_ref.init(m_nextUnwrappedSetpoint.model());
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
        if (!m_validSetpoint)
            return false;
        double positionError = MathUtil.angleModulus(m_nextUnwrappedSetpoint.x() - m_mechanism.getWrappedPositionRad());
        double velocityError = m_nextUnwrappedSetpoint.v() - m_mechanism.getVelocityRad_S();
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
        m_unwrappedGoal = null;
        m_nextUnwrappedSetpoint = null;
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
    double nearMeasurement(double unwrappedPositionRad) {
        double unwrappedMeasurement = m_mechanism.getUnwrappedPositionRad();
        return MathUtil.angleModulus(unwrappedPositionRad - unwrappedMeasurement) + unwrappedMeasurement;
    }

}
