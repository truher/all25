package org.team100.lib.motor;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.CotemporalCache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

/**
 * Relies on Memo and Takt, so you must put Memo.resetAll() and Takt.update() in
 * Robot.robotPeriodic().
 */
public class SimulatedBareMotor implements BareMotor {
    private static final boolean DEBUG = false;

    private final double m_freeSpeedRad_S;

    private final LoggerFactory m_log;
    private final DoubleLogger m_log_duty;
    private final DoubleLogger m_log_velocityInput;
    private final DoubleLogger m_log_positionInput;
    private final CotemporalCache<Model100> m_stateCache;

    // just like in a real motor, the inputs remain until zeroed by the watchdog.
    // nullable; only one (velocity or position) is used at a time.
    private Double m_velocityInput;
    private Double m_positionInput;

    private Model100 m_state = new Model100();

    private double m_time = Takt.get();

    public SimulatedBareMotor(LoggerFactory parent, double freeSpeedRad_S) {
        m_log = parent.type(this);
        m_freeSpeedRad_S = freeSpeedRad_S;
        m_log_duty = m_log.doubleLogger(Level.DEBUG, "duty_cycle");
        m_log_velocityInput = m_log.doubleLogger(Level.DEBUG, "velocity input");
        m_log_positionInput = m_log.doubleLogger(Level.DEBUG, "position input");
        m_stateCache = Cache.of(this::update);
    }

    private Model100 update() {
        if (DEBUG) {
            Object[] args = { m_log.getRoot() };
            System.out.printf("motor %s update\n", args);
        }
        double dt = dt();
        if (DEBUG) {
            Object[] args1 = { dt };
            System.out.printf("SimulatedBareMotor dt %f\n", args1);
        }
        if (m_velocityInput != null) {
            if (DEBUG) {
                Object[] args2 = { m_velocityInput };
                System.out.printf("SimulatedBareMotor v %f\n", args2);
            }
            if (dt > 0.04) {
                // probably we should not extrapolate
                m_state = new Model100(m_state.x(), m_velocityInput);
            } else {
                m_state = new Model100(m_state.x() + m_velocityInput * dt, m_velocityInput);
            }
        }
        if (m_positionInput != null) {
            if (DEBUG) {
                Object[] args2 = { m_positionInput };
                System.out.printf("SimulatedBareMotor x %f\n", args2);
            }
            if (dt < 0.01) {
                // probably we should not differentiate
                m_state = new Model100(m_positionInput, m_state.v());
            } else {
                m_state = new Model100(m_positionInput, (m_positionInput - m_state.x()) / dt);
            }
        }
        if (DEBUG) {
            Object[] args2 = { m_state };
            System.out.printf("SimulatedBareMotor state %s\n", args2);
        }
        return m_state;
    }

    double dt() {
        double now = Takt.get();
        double dt = now - m_time;
        m_time = now;
        return dt;
    }

    @Override
    public void setDutyCycle(double dutyCycle) {
        final double output = MathUtil.clamp(
                Util.notNaN(dutyCycle), -1, 1);
        m_log_duty.log(() -> output);
        setVelocity(output * m_freeSpeedRad_S, 0, 0);
    }

    /** ignores accel and torque */
    @Override
    public void setVelocity(double velocityRad_S, double accelRad_S2, double torqueNm) {
        if (DEBUG) {
            Object[] args = { m_log.getRoot(), velocityRad_S };
            System.out.printf("motor %s set velocity %6.3f\n", args);
        }
        m_velocityInput = MathUtil.clamp(
                Util.notNaN(velocityRad_S), -m_freeSpeedRad_S, m_freeSpeedRad_S);
        // you can't use velocity and position control at the same time
        m_positionInput = null;
    }

    /** ignores velocity and torque */
    @Override
    public void setUnwrappedPosition(double position, double velocity, double accel, double torque) {
        if (DEBUG) {
            Object[] args = { m_log.getRoot(), position };
            System.out.printf("motor %s set position %6.3f\n", args);
        }
        m_positionInput = position;
        // you can't use velocity and position control at the same time
        m_velocityInput = null;
    }

    /** placeholder */
    @Override
    public double kROhms() {
        return 0.1;
    }

    /** placeholder */
    @Override
    public double kTNm_amp() {
        return 0.02;
    }

    @Override
    public void stop() {
        m_velocityInput = 0.0;
    }

    @Override
    public void close() {
        //
    }

    @Override
    public double getVelocityRad_S() {
        return m_stateCache.get().v();
    }

    @Override
    public double getCurrent() {
        // this is totally wrong
        return getVelocityRad_S() / 10.0;
    }

    public double getUnwrappedPositionRad() {
        double pos = m_stateCache.get().x();
        if (Double.isNaN(pos))
            throw new IllegalArgumentException("motor pos");
        return pos;
    }

    /** resets the caches, so the new value is immediately available. */
    @Override
    public void setUnwrappedEncoderPositionRad(double positionRad) {
        if (Double.isNaN(positionRad))
            throw new IllegalArgumentException("motor set position");
        m_positionInput = positionRad;
        m_stateCache.reset();
    }

    @Override
    public void setTorqueLimit(double torqueNm) {
        //
    }

    @Override
    public void periodic() {
        if (m_positionInput != null)
            m_log_positionInput.log(() -> m_positionInput);
        if (m_velocityInput != null)
            m_log_velocityInput.log(() -> m_velocityInput);
    }

    /** resets the caches, so the new value is immediately available. */
    public void reset() {
        m_positionInput = 0.0;
        m_velocityInput = 0.0;
        m_time = Takt.get();
        m_stateCache.reset();
    }
}
