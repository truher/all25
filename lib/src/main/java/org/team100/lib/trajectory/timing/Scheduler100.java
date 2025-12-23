package org.team100.lib.trajectory.timing;

import java.util.List;

import org.team100.lib.util.Math100;

/**
 * Assign timestamps waypoints.
 * 
 * This is the same approach as the old ScheduleGenerator: assign durations to
 * the arcs between waypoints, using constraints on pathwise velocity and
 * accel/decel, walk over the waypoints once forward, and
 * once backward, then once more to add up the durations.
 */
public class Scheduler100 {

    public static interface Constraint {
        /**
         * Can depend on curvature and course and heading and heading rate,
         * none of which vary during scheduling.
         * 
         * @return a positive number.
         */
        double vmax(double q);

        /**
         * Since v is alwasy positive, amin means decel.
         * 
         * Can depend on pose and curvature and heading rate, and also pathwise
         * velocity. of these only pathwise velocity changes during the scheduling.
         * 
         * @returns a negative number
         */
        double amin(double q, double v);

        /**
         * Since v is always positive, amax means accel.
         * 
         * Can depend on pose and curvature and heading rate, and also pathwise
         * velocity. of these only pathwise velocity changes during the scheduling.
         *
         * @return a positive number
         */
        double amax(double q, double v);
    }

    private final Constraint m_c;

    public Scheduler100(Constraint c) {
        m_c = c;
    }

    public double[] schedule(double[] q) {

        int n = q.length - 1;

        // initial dt
        // new dt is never less than this so choose a small number
        double[] dt = new double[n + 1];
        for (int i = 0; i < n; ++i) {
            dt[i] = 0.10 / n;
        }

        // velocity constraint
        for (int i = 1; i < n + 1; ++i) {
            // dx is never negative
            double dx = q[i] - q[i - 1];
            // v is never negative
            double v = dx / dt[i - 1];
            double vmax = m_c.vmax(q[i]);
            double newdt = dx / vmax;
            // only slower
            dt[i - 1] = Math.max(dt[i - 1], newdt);
        }

        // accel constraint forward, using backward finite differences
        for (int i = 2; i < n + 1; ++i) {
            double v0 = (q[i - 1] - q[i - 2]) / dt[i - 2];
            double dx = q[i] - q[i - 1];
            while (true) {
                double v = dx / dt[i - 1];
                double a = (v - v0) / dt[i - 1];
                if (a > 0) {
                    double amax = m_c.amax(q[i], v);
                    if (a > amax) {
                        double newdt = solve(amax, v0, -dx);
                        // only slower
                        if (newdt > dt[i - 1]) {
                            dt[i - 1] = newdt;
                            continue;
                        }
                    }
                } else if (a < 0) {
                    double amin = m_c.amin(q[i], v);
                    if (a < amin) {
                        double newdt = solve(amin, v0, -dx);
                        // only slower
                        if (newdt > dt[i - 1]) {
                            dt[i - 1] = newdt;
                            continue;
                        }
                    }
                }
                break;
            }
        }

        // accel constraint backward, using forward finite differences
        for (int i = n - 2; i >= 0; --i) {
            double v0 = (q[i + 2] - q[i + 1]) / dt[i + 1];
            double dx = q[i + 1] - q[i];
            while (true) {
                double v = dx / dt[i];
                double a = (v0 - v) / dt[i];
                if (a > 0) {
                    double amax = m_c.amax(q[i], v);
                    if (a > amax) {
                        double newdt = solve(amax, -v0, dx);
                        // only slower
                        if (newdt > dt[i]) {
                            dt[i] = newdt;
                            continue;
                        }
                    }
                } else if (a < 0) {
                    double amin = m_c.amin(q[i], v);
                    if (a < amin) {
                        double newdt = solve(amin, -v0, dx);
                        // only slower
                        if (newdt > dt[i]) {
                            dt[i] = newdt;
                            continue;
                        }
                    }
                }
                break;
            }
        }
        return dt;
    }

    private static double solve(double A, double B, double C) {
        List<Double> soln = Math100.solveQuadratic(A, B, C);
        return choose(soln);
    }

    /**
     * choose smallest non-negative solution
     * dt is never negative ... and i think also should never be zero ...
     */
    private static double choose(List<Double> soln) {
        double x0 = Double.POSITIVE_INFINITY;
        for (double x : soln) {
            if (x >= 0 && x < x0)
                x0 = x;
        }
        if (Double.isFinite(x0))
            return x0;

        // System.out.println("no solution");
        return 0;

        // throw new IllegalArgumentException();
    }

}
