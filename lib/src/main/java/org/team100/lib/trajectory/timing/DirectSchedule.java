package org.team100.lib.trajectory.timing;

import org.team100.lib.trajectory.path.spline.SplineR1;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/**
 * Experiment with schedule map, a stand-off function for the spline parameter
 * as a function of time.
 * 
 * Uses https://en.wikipedia.org/wiki/Finite_difference
 * 
 * This matches the notation in README.md
 */
public class DirectSchedule {
    private static final double EPSILON = 1e-6;

    private final SplineR1 m_spline;
    private final InterpolatingDoubleTreeMap m_sdot;
    // integrates sdot
    private final InterpolatingDoubleTreeMap m_s;
    private double m_maxT;

    public DirectSchedule(SplineR1 spline) {
        m_spline = spline;
        m_sdot = new InterpolatingDoubleTreeMap();
        m_s = new InterpolatingDoubleTreeMap();
    }

    public void put(double t, double sdot) {
        m_sdot.put(t, sdot);
        m_maxT = Math.max(m_maxT, t);
        // keep s in sync
        integrate();
    }

    private void integrate() {
        double s = 0;
        m_s.clear();
        double dt = 0.001;
        m_s.put(0.0, 0.0);
        for (double t = dt; t <= m_maxT; t += dt) {
            double sdot = sdot(t);
            s += sdot * dt;
            m_s.put(t, s);
        }
    }

    /** position for time t */
    public double x(double t) {
        return q(s(t));
    }

    public double v(double t) {
        return qprime(s(t)) * sdot(t);
    }

    public double a(double t) {
        return qprimeprime(s(t)) * sdot(t) * sdot(t) + qprime(s(t)) * sdotdot(t);
    }

    /** position for parameter s */
    public double q(double s) {
        s = MathUtil.clamp(s, 0, 1);
        return m_spline.getPosition(s);
    }

    /** dq/ds, derivative of position wrt parameter s */
    public double qprime(double s) {
        s = MathUtil.clamp(s, 0, 1);
        return m_spline.getVelocity(s);
    }

    /** d^2q/ds^2, second derivative of postion wrt parameter s */
    public double qprimeprime(double s) {
        s = MathUtil.clamp(s, 0, 1);
        return m_spline.getAcceleration(s);
    }

    /** spline parameter, s, for time t */
    public double s(double t) {
        return m_s.get(t);
    }

    public double sdot(double t) {
        return m_sdot.get(t);
    }

    public double sdotdot(double t) {
        return (sdot(t + EPSILON) - sdot(t)) / EPSILON;
    }

}
