package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.util.Math100;

import edu.wpi.first.math.MathUtil;

/**
 * Represents a state within a 2d holonomic trajectory, i.e. with heading
 * independent from course.
 * 
 * The timing fields are set by the ScheduleGenerator.
 */
public class TimedPose {
    private static final boolean DEBUG = false;
    private final Pose2dWithMotion m_state;
    /** Time we achieve this state. */
    private final double m_timeS;
    /** ds/dt */
    private final double m_velocityM_S;
    /** d^2s/dt^2 */
    private double m_accelM_S_S;

    public TimedPose(
            Pose2dWithMotion state,
            double t,
            double velocity,
            double acceleration) {
        m_state = state;
        m_timeS = t;
        m_velocityM_S = velocity;
        m_accelM_S_S = acceleration;
    }

    public Pose2dWithMotion state() {
        return m_state;
    }

    public double getTimeS() {
        return m_timeS;
    }

    public double velocityM_S() {
        return m_velocityM_S;
    }

    /** accel is set based on the velocity of the next state, so we set it here. */
    void set_acceleration(double acceleration) {
        m_accelM_S_S = acceleration;
    }

    /** this means acceleration along the path, not centripetal acceleration. */
    public double acceleration() {
        return m_accelM_S_S;
    }

    @Override
    public String toString() {
        return String.format("state %s, time %5.3f, vel %5.3f, acc %5.3f",
                m_state,
                m_timeS,
                m_velocityM_S,
                m_accelM_S_S);
    }

    public TimedPose interpolate2(TimedPose other, double x) {
        final double new_t = MathUtil.interpolate(m_timeS, other.m_timeS, x);
        final double delta_t = new_t - getTimeS();
        if (delta_t < 0.0) {
            return other.interpolate2(this, 1.0 - x);
        }
        boolean reversing = m_velocityM_S < 0.0 || (Math.abs(m_velocityM_S - 0.0) <= 1e-12 && acceleration() < 0.0);
        final double new_v = m_velocityM_S + m_accelM_S_S * delta_t;
        final double new_s = (reversing ? -1.0 : 1.0)
                * (m_velocityM_S * delta_t + .5 * m_accelM_S_S * delta_t * delta_t);

        double interpolant = new_s / m_state.distanceM(other.m_state);
        if (Double.isNaN(interpolant)) {
            interpolant = 1.0;
        }

        return new TimedPose(
                m_state.interpolate(other.m_state, interpolant),
                new_t,
                new_v,
                m_accelM_S_S);
    }

    public double distance(TimedPose other) {
        return m_state.distanceM(other.m_state);
    }

    @Override
    public boolean equals(final Object other) {
        if (!(other instanceof TimedPose)) {
            if (DEBUG)
                System.out.println("wrong type");
            return false;
        }
        TimedPose ts = (TimedPose) other;
        if (!m_state.equals(ts.m_state)) {
            if (DEBUG)
                System.out.println("wrong state");
            return false;
        }
        if (!Math100.epsilonEquals(m_timeS, ts.m_timeS)) {
            if (DEBUG)
                System.out.println("wrong time");
            return false;
        }
        return true;
    }
}
