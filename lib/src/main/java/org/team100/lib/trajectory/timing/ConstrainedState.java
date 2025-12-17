package org.team100.lib.trajectory.timing;

import java.util.List;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.trajectory.timing.TimingConstraint.NonNegativeDouble;
import org.team100.lib.util.Math100;

class ConstrainedState {
    // using MAX_VALUE tickles some bugs
    private static final double MAX_V = 100;
    private static final double MAX_A = 100;

    private final Pose2dWithMotion m_state;
    /** Cumulative distance along the path */
    private final double m_distanceM;

    private double m_velocityM_S;
    private double m_minAccelM_S2;
    private double m_maxAccelM_S2;

    public ConstrainedState(Pose2dWithMotion state, double distance) {
        m_state = state;
        m_distanceM = distance;
        setVelocityM_S(MAX_V);
        setMinAccel(-MAX_A);
        setMaxAccel(MAX_A);
    }

    /**
     * Clamp state velocity to constraints.
     */
    public void clampVelocity(List<TimingConstraint> constraints) {
        for (TimingConstraint constraint : constraints) {
            NonNegativeDouble constraintVel = constraint.getMaxVelocity(m_state);
            double value = constraintVel.getValue();
            setVelocityM_S(Math.min(getVelocityM_S(), value));
        }
    }

    /**
     * Clamp constraint state accelerations to the constraints.
     */
    public void clampAccel(List<TimingConstraint> constraints) {
        for (TimingConstraint constraint : constraints) {
            TimingConstraint.MinMaxAcceleration min_max_accel = constraint
                    .getMinMaxAcceleration(m_state, getVelocityM_S());
            double minAccel = Math100.notNaN(min_max_accel.getMinAccel());
            m_minAccelM_S2 = Math.max(m_minAccelM_S2, minAccel);
            double maxAccel = Math100.notNaN(min_max_accel.getMaxAccel());
            m_maxAccelM_S2 = Math.min(m_maxAccelM_S2, maxAccel);
        }

    }

    public Pose2dWithMotion getState() {
        return m_state;
    }

    /** this should be L2 distance */
    public double getDistanceM() {
        return m_distanceM;
    }

    /**
     * if we're using L2 norm then this isn't meters and isn't always even cartesian
     * at all
     */
    public double getVelocityM_S() {
        return m_velocityM_S;
    }

    public void setVelocityM_S(double velocityM_S) {
        if (Double.isNaN(velocityM_S))
            throw new IllegalArgumentException();
        m_velocityM_S = velocityM_S;
    }

    public double getMinAccel() {
        return m_minAccelM_S2;
    }

    public void setMinAccel(double minAccelM_S2) {
        m_minAccelM_S2 = minAccelM_S2;
    }

    public double getMaxAccel() {
        return m_maxAccelM_S2;
    }

    public void setMaxAccel(double maxAccelM_S2) {
        m_maxAccelM_S2 = maxAccelM_S2;
    }

    @Override
    public String toString() {
        return m_state.toString() + ", distance: " + m_distanceM + ", vel: " + getVelocityM_S() + ", " +
                "min_acceleration: " + m_minAccelM_S2 + ", max_acceleration: " + m_maxAccelM_S2;
    }
}