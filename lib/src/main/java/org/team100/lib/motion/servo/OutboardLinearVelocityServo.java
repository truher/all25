package org.team100.lib.motion.servo;

import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;

/** There is no profile here. */
public class OutboardLinearVelocityServo implements LinearVelocityServo {
    private static final boolean DEBUG = false;
    private static final double VELOCITY_TOLERANCE = 1;

    private final LinearMechanism m_mechanism;
    private final DoubleLogger m_log_setpoint_v;
    private final DoubleLogger m_log_setpoint_a;

    // for calculating acceleration
    private double m_prevGoal = 0;
    private double m_goal;

    public OutboardLinearVelocityServo(LoggerFactory parent, LinearMechanism mechanism) {
        LoggerFactory child = parent.type(this);
        m_mechanism = mechanism;
        m_log_setpoint_v = child.doubleLogger(Level.TRACE, "setpoint v (m_s)");
        m_log_setpoint_a = child.doubleLogger(Level.TRACE, "setpoint a (m_s2)");
    }

    @Override
    public void reset() {
        if (DEBUG)
            System.out.println("WARNING: make sure resetting encoder position doesn't break anything");
        m_mechanism.resetEncoderPosition();
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        m_mechanism.setDutyCycle(dutyCycle);
    }

    /** Passthrough to the outboard control. */
    @Override
    public void setVelocity(double setpointM_S) {
        setVelocity(setpointM_S, accel(setpointM_S));
    }

    /** Passthrough to the outboard control. */
    @Override
    public void setVelocity(double setpointM_S, double setpointM_S2) {
        if (DEBUG) {
            System.out.printf("setpointM_S %6.3f\n", setpointM_S);
        }
        m_goal = setpointM_S;
        m_mechanism.setVelocity(setpointM_S, setpointM_S2, 0);
        m_log_setpoint_v.log(() -> setpointM_S);
        m_log_setpoint_a.log(() -> setpointM_S2);
    }

    /**
     * @return Current velocity measurement. Note this can be noisy, maybe filter
     *         it.
     */
    @Override
    public double getVelocity() {
        return m_mechanism.getVelocityM_S();
    }

    @Override
    public boolean atGoal() {
        return Math.abs(m_goal - m_mechanism.getVelocityM_S()) < VELOCITY_TOLERANCE;
    }

    @Override
    public void stop() {
        m_mechanism.stop();
    }

    @Override
    public double getDistance() {
        return m_mechanism.getPositionM();
    }

    @Override
    public void periodic() {
        m_mechanism.periodic();
    }

    ////////////////////////////////////////////////

    /**
     * Acceleration from trailing difference in velocity.
     * 
     * To avoid injecting clock noise into the acceleration signal, this uses
     * a constant dt, TimedRobot100.LOOP_PERIOD_S, so you'd better be calling this
     * at about that rate.
     * 
     * @param setpoint desired velocity
     */
    private double accel(double setpoint) {
        double accel = (setpoint - m_prevGoal) / TimedRobot100.LOOP_PERIOD_S;
        m_prevGoal = setpoint;
        return accel;
    }
}
