package org.team100.lib.trajectory.timing;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.trajectory.path.spline.SplineR1;
import org.team100.lib.util.Math100;

public class DirectScheduleTest {
    private static final double EPSILON = 1e-6;

    @Test
    void testSimple0() {
        // constant velocity
        // we want v=0 at the end, so the map here goes to zero at the end.
        SplineR1 spline = SplineR1.get(0, 1, 1, 1, 0, 0);
        DirectSchedule schedule = new DirectSchedule(spline);
        // by hand, for amax = 2, vmax = 0.5
        schedule.put(-100.0, 1.0);
        schedule.put(0.0, 0.0);
        schedule.put(0.1, 0.2);
        schedule.put(0.2, 0.4);
        schedule.put(0.3, 0.5);
        schedule.put(0.4, 0.5);
        schedule.put(0.5, 0.5);
        schedule.put(0.6, 0.5);
        schedule.put(0.7, 0.5);
        schedule.put(0.8, 0.5);
        schedule.put(0.9, 0.5);
        schedule.put(1.0, 0.5);
        schedule.put(1.1, 0.5);
        schedule.put(1.2, 0.5);
        schedule.put(1.3, 0.5);
        schedule.put(1.4, 0.5);
        schedule.put(1.5, 0.5);
        schedule.put(1.6, 0.5);
        schedule.put(1.7, 0.5);
        schedule.put(1.8, 0.5);
        schedule.put(1.9, 0.5);
        schedule.put(2.0, 0.5);
        schedule.put(2.1, 0.4);
        schedule.put(2.2, 0.2);
        schedule.put(2.3, 0.0);
        schedule.put(2.4, 0.0);
        schedule.put(100.0, 0.0);
        System.out.println("t, x, v, a");
        for (double t = 0; t <= 3.001; t += 0.1) {
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f\n",
                    t, schedule.x(t), schedule.v(t), schedule.a(t));
        }
    }

    @Test
    void testSimple1() {
        // dx = 0 at the ends
        // we also want 0 at the ends, so the map is constant there
        SplineR1 spline = SplineR1.get(0, 1, 0, 0, 0, 0);
        DirectSchedule schedule = new DirectSchedule(spline);
        // by hand, for amax = 2, vmax = 0.5
        schedule.put(-100.0, 1.0);
        schedule.put(0.0, 0.75);
        schedule.put(0.1, 0.75);
        schedule.put(0.2, 0.6);
        schedule.put(0.3, 0.58);
        schedule.put(0.4, 0.46);
        schedule.put(0.5, 0.3);
        schedule.put(0.6, 0.3);
        schedule.put(0.7, 0.3);
        schedule.put(0.8, 0.3);
        schedule.put(0.9, 0.3);
        schedule.put(1.0, 0.3);
        schedule.put(1.1, 0.3);
        schedule.put(1.2, 0.3);
        schedule.put(1.3, 0.3);
        schedule.put(1.4, 0.3);
        schedule.put(1.5, 0.3);
        schedule.put(1.6, 0.3);
        schedule.put(1.7, 0.4);
        schedule.put(1.8, 0.4);
        schedule.put(1.9, 0.4);
        schedule.put(2.0, 0.5);
        schedule.put(2.1, 0.65);
        schedule.put(2.2, 0.75);
        schedule.put(2.3, 0.75);
        schedule.put(2.4, 0.75);
        schedule.put(2.5, 0.75);
        schedule.put(2.6, 0.75);
        schedule.put(2.7, 0.75);
        schedule.put(2.8, 0.75);
        schedule.put(100.0, 0.0);
        System.out.println("t, x, v, a, qprime");
        for (double t = 0; t <= 3.001; t += 0.1) {
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                    t, schedule.x(t), schedule.v(t), schedule.a(t),
                    schedule.qprime(schedule.s(t)));
        }
    }

    @Test
    void testSimple3() {
        SplineR1 spline = SplineR1.get(0, 1, 0, 0, 0, 0);

        double DS = 0.01;
        double qq = 0;
        System.out.println("s, q, qq, qError, qprime, qprimeprime");
        for (double s = 0; s <= 1.001; s += DS) {
            double q = spline.getPosition(s);
            double qprime = spline.getVelocity(s);
            // https://en.wikipedia.org/wiki/Numerical_integration
            if (s > 0) {
                // trapezoid integration is good to about 50 ppm
                // qq += (spline.getVelocity(s - DS) + spline.getVelocity(s)) * DS/2;
                // simpsons rule is good to 2 ppb
                qq += (spline.getVelocity(s - DS)
                        + 4 * spline.getVelocity(s - DS / 2)
                        + spline.getVelocity(s)) * DS / 6;
            }
            double qError = q - qq;
            double qprimeprime = spline.getAcceleration(s);
            System.out.printf("%10.8f, %10.8f, %10.8f, %12.10f, %10.8f, %10.8f\n",
                    s, q, qq, qError, qprime, qprimeprime);
        }
    }

    /**
     * qdot constraint varies; for now it's a function of s.
     * 
     * In reality, depends on curvature and course and heading and heading rate,
     * none of which vary during scheduling.
     * 
     * always positive.
     */
    double qdotmax(double q) {
        // if (q > 0.45 && q < 0.55)
        //     return 0.5;
        return 1.0;
    }

    /**
     * a positive number
     * 
     * In reality, depends on pose and curvature and heading rate, and also pathwise
     * velocity. of these only pathwise velocity changes during the scheduling.
     */
    double qdotdotmax(double q, double v) {
        if (q < 0.02) // soft start
            return q*10;
        return 5.0 - v;
    }

    /**
     * Since v is never negative, qdotdotmin always means deceleration.
     * 
     * In reality, depends on pose and curvature and heading rate, and also pathwise
     * velocity. of these only pathwise velocity changes during the scheduling.
     * 
     * @returns a negative number
     */
    double qdotdotmin(double s, double v) {
        // if (s > 1.4) // soft stop
        //     return (1 - s) * -50 + v;
        return -10.0 + v;
    }

    @Test
    void testSimple4() {
        int n = 100;
        SplineR1 q = SplineR1.get(0, 1, 0, 0, 0, 0);

        // these are the spline sample points; they can be
        // in arbitrary locations. s is immutable.
        double[] s = new double[n + 1];
        for (int i = 0; i < n + 1; ++i) {
            s[i] = (double) i / n;
        }
        // start with initial estimate of sdot
        double[] sdot = new double[n + 1];
        for (int i = 0; i < n + 1; ++i) {
            sdot[i] = 1.0;
        }

        // velocity constraint
        for (int i = 0; i < n + 1; ++i) {
            // first derivative of q wrt parameter s
            double qprimei = q.getVelocity(s[i]);
            // second derivative of q wrt parameter s
            double qprimeprimei = q.getAcceleration(s[i]);
            // first derivative of q wrt time, using chain rule
            // qdot = dq/ds * ds/dt
            double qdoti = qprimei * sdot[i];
            // also sdot(t) = 1/tprime(s)
            // adjust sdot so that qdot is under the constraint
            double qdotmaxi = qdotmax(s[i]);
            if (Math.abs(qdoti) > qdotmaxi) {
                sdot[i] = qdotmaxi / qprimei;
            }
        }

        // accel constraint
        for (int i = 1; i < n + 1; ++i) {
            // previous
            int j = i - 1;
            // first derivative of q wrt parameter s
            double qprimei = q.getVelocity(s[i]);
            // second derivative of q wrt parameter s
            double qprimeprimei = q.getAcceleration(s[i]);

            double ds = s[i] - s[j];

            // trailing difference to get tprimeprime
            double tprimeprimei = (1 / sdot[i] - 1 / sdot[j]) / ds;
            // differentiate sdot to get sdotdot.
            // d2s/dt2 = - d2t/ds2 * (ds/dt)^3
            // sdotdot = -tprimeprime * sdot^3
            double sdotdoti = -tprimeprimei * sdot[i] * sdot[i] * sdot[i];

            // chain rule
            // d2q/dt2 = d2q/ds2 * (ds/dt)^2 + dq/ds * d2s/dt2
            // qdotdot = qprimeprime * sdot^2 + qprime * sdotdot
            double qdotdoti = qprimeprimei * sdot[i] * sdot[i] + qprimei * sdotdoti;
            // adjust sdot so that qdotdot is under the constraint
            double qdotdotmaxi = qdotdotmax(s[i], 0);
            if (Math.abs(qdotdoti) > Math.abs(qdotdotmaxi)) {
                double sdotnew = Math.sqrt((2 * qdotdotmaxi * ds + qprimei * sdot[j] * sdot[j])
                        / (2 * qprimeprimei * ds + qprimei));
                if (sdotnew < sdot[i]) {
                    sdot[i] = sdotnew;
                    tprimeprimei = (1 / sdot[i] - 1 / sdot[j]) / ds;
                    sdotdoti = -tprimeprimei * sdot[i] * sdot[i] * sdot[i];
                }
            }
        }

        // accel constraint backwards
        for (int i = n - 1; i > 0; --i) {
            // previous
            int j = i + 1;
            // first derivative of q wrt parameter s
            // note negative sign
            double qprimei = -q.getVelocity(s[i]);
            // second derivative of q wrt parameter s
            // note negative sign
            double qprimeprimei = -q.getAcceleration(s[i]);

            // this is a negative number
            double ds = s[i] - s[j];

            // trailing difference to get tprimeprime
            // this is a positive number
            double tprimeprimei = (1 / sdot[i] - 1 / sdot[j]) / ds;
            // differentiate sdot to get sdotdot.
            // d2s/dt2 = - d2t/ds2 * (ds/dt)^3
            // sdotdot = -tprimeprime * sdot^3
            double sdotdoti = -tprimeprimei * sdot[i] * sdot[i] * sdot[i];

            // chain rule
            // d2q/dt2 = d2q/ds2 * (ds/dt)^2 + dq/ds * d2s/dt2
            // qdotdot = qprimeprime * sdot^2 + qprime * sdotdot
            double qdotdoti = qprimeprimei * sdot[i] * sdot[i] + qprimei * sdotdoti;
            // adjust sdot so that qdotdot is under the constraint
            double qdotdotmaxi = qdotdotmax(s[i], 0);
            if (Math.abs(qdotdoti) > Math.abs(qdotdotmaxi)) {
                double sdotnew = Math.sqrt((2 * qdotdotmaxi * ds + qprimei * sdot[j] * sdot[j])
                        / (2 * qprimeprimei * ds + qprimei));
                if (sdotnew < sdot[i]) {
                    sdot[i] = sdotnew;
                    tprimeprimei = (1 / sdot[i] - 1 / sdot[j]) / ds;
                    sdotdoti = -tprimeprimei * sdot[i] * sdot[i] * sdot[i];
                }
            }
        }

        // integrate and dump the result
        System.out.println("t, s, sdot, sdotdot, q, qdot, qdotdot");
        double t = 0;
        for (int i = 0; i < n + 1; ++i) {

            double qi = q.getPosition(s[i]);
            double qprimei = q.getVelocity(s[i]);
            double qprimeprimei = q.getAcceleration(s[i]);
            double qdoti = qprimei * sdot[i];
            double sdotdoti = 0;
            double qdotdoti = 0;
            if (i > 0) {
                int j = i - 1;
                // Integrate.
                // We have sdot(t) but we want to integrate over s to find t
                // The derivative of the inverse is the reciprocal
                // https://en.wikipedia.org/wiki/Inverse_function_rule
                // sdot(t) = 1/tprime(s)
                double ds = s[i] - s[j];
                // tprime = dt/ds
                double tprimei = 1 / sdot[i];
                double dt1 = tprimei * ds;

                // trailing difference to get tprimeprime
                double tprimeprimei = (1 / sdot[i] - 1 / sdot[j]) / ds;
                // differentiate sdot to get sdotdot.
                // d2s/dt2 = - d2t/ds2 * (ds/dt)^3
                sdotdoti = -tprimeprimei * sdot[i] * sdot[i] * sdot[i];
                qdotdoti = qprimeprimei * sdot[i] * sdot[i] + qprimei * sdotdoti;

                t += dt1;
            }
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f, %5.3f\n",
                    t, s[i], sdot[i], sdotdoti, qi, qdoti, qdotdoti);
        }
    }

    /**
     * schedules dt directly, doesn't try to us spline velocity etc.
     * 
     * makes no attempt to limit jerk; jerk limiting would require an iterative
     * solver, which would be nice to avoid, and it's not that important
     * in practice.
     */
    @Test
    void testSimple5() {
        SplineR1 spline = SplineR1.get(0, 1, 0, 0, 0, 0);
        // these are the spline sample points; they can be
        // in arbitrary locations. s is immutable.
        double[] s = new double[101];
        for (int i = 0; i < 101; ++i) {
            s[i] = (double) i / 100;
        }

        double[] q = new double[s.length];
        for (int i = 0; i < s.length; ++i) {
            q[i] = spline.getPosition(s[i]);
        }

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
            double vmax = qdotmax(q[i]);
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
                    double amax = qdotdotmax(q[i], v);
                    if (a > amax) {
                        double newdt = solve(amax, v0, -dx);
                        // only slower
                        if (newdt > dt[i - 1]) {
                            dt[i - 1] = newdt;
                            continue;
                        }
                    }
                } else if (a < 0) {
                    double amin = qdotdotmin(q[i], v);
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
                    double amax = qdotdotmax(q[i], v);
                    if (a > amax) {
                        double newdt = solve(amax, -v0, dx);
                        // only slower
                        if (newdt > dt[i]) {
                            dt[i] = newdt;
                            continue;
                        }
                    }
                } else if (a < 0) {
                    double amin = qdotdotmin(q[i], v);
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

        // integrate and dump the result
        System.out.println("t, x, v, a");
        double t = 0;
        for (int i = 0; i < n + 1; ++i) {
            double x = q[i];
            double v = 0;
            double a = 0;
            if (i > 0) {
                t += dt[i - 1];
            }
            // compute v using backward finite difference
            if (i > 0) {
                double dx = q[i] - q[i - 1];
                v = dx / dt[i - 1];
            }
            // compute a using backward finite difference
            if (i > 1) {
                double dx0 = q[i] - q[i - 1];
                double v0 = dx0 / dt[i - 1];
                double dx1 = q[i - 1] - q[i - 2];
                double v1 = dx1 / dt[i - 2];
                a = (v0 - v1) / dt[i - 1];
            }
            System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f\n",
                    t, x, v, a);
        }
    }

    private double solve(double A, double B, double C) {
        List<Double> soln = Math100.solveQuadratic(A, B, C);
        return choose(soln);
    }

    /**
     * choose smallest non-negative solution
     * dt is never negative ... and i think also should never be zero ...
     */
    private double choose(List<Double> soln) {
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
