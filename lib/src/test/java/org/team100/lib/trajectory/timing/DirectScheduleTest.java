package org.team100.lib.trajectory.timing;

import org.junit.jupiter.api.Test;
import org.team100.lib.trajectory.path.spline.SplineR1;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

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
    void testSimple2() {
        SplineR1 spline = SplineR1.get(0, 1, 0, 0, 0, 0);
        InterpolatingDoubleTreeMap m = new InterpolatingDoubleTreeMap();
        for (double s = 0; s <= 1; s += 0.05) {
            double q = spline.getPosition(s);
            m.put(q, s);
        }
        System.out.println("q, s");
        for (double q = 0; q <= 1.001; q += 0.01) {
            double s = m.get(q);
            double qprime = spline.getVelocity(s);
            double qprimeprime = spline.getAcceleration(s);
            System.out.printf("%5.3f, %5.3f, %5.3f\n", q, s);
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

    /** qdot constraint varies; for now it's a function of s */
    double qdotmax(double s) {
        if (s > 0.4 && s < 0.6)
            return 0.5;
        return 1.0;
    }

    double qdotdotmax(double s) {
        return 5.0;
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

            double sdotdoti = 0;
            // trailing difference to get tprimeprime
            double tprimeprimei = (1 / sdot[i] - 1 / sdot[j]) / ds;
            // differentiate sdot to get sdotdot.
            // d2s/dt2 = - d2t/ds2 * (ds/dt)^3
            // sdotdot = -tprimeprime * sdot^3
            sdotdoti = -tprimeprimei * sdot[i] * sdot[i] * sdot[i];

            // chain rule
            // d2q/dt2 = d2q/ds2 * (ds/dt)^2 + dq/ds * d2s/dt2
            // qdotdot = qprimeprime * sdot^2 + qprime * sdotdot
            double qdotdoti = qprimeprimei * sdot[i] * sdot[i] + qprimei * sdotdoti;
            // adjust sdot so that qdotdot is under the constraint
            double qdotdotmaxi = qdotdotmax(s[i]);
            if (Math.abs(qdotdoti) > qdotdotmaxi) {
                sdotdoti = (qdotdotmaxi - qprimeprimei * sdot[i] * sdot[i]) / qprimei;
                // integrate to find sdot
                // sdot = ds/dt
                // dt = ds/sdot
                double sdotnew = sdot[j] + sdotdoti * ds / sdot[i];
                if (Math.abs(sdotnew) < Math.abs(sdot[i]))
                    sdot[i] = sdotnew;
            }
        }

        // accel constraint backwards
        for (int i = n-1; i > 0; --i) {
            // previous
            int j = i + 1;
            // first derivative of q wrt parameter s
            double qprimei = q.getVelocity(s[i]);
            // second derivative of q wrt parameter s
            double qprimeprimei = q.getAcceleration(s[i]);

            double ds = s[i] - s[j];

            double sdotdoti = 0;
            // trailing difference to get tprimeprime
            double tprimeprimei = (1 / sdot[i] - 1 / sdot[j]) / ds;
            // differentiate sdot to get sdotdot.
            // d2s/dt2 = - d2t/ds2 * (ds/dt)^3
            // sdotdot = -tprimeprime * sdot^3
            sdotdoti = -tprimeprimei * sdot[i] * sdot[i] * sdot[i];

            // chain rule
            // d2q/dt2 = d2q/ds2 * (ds/dt)^2 + dq/ds * d2s/dt2
            // qdotdot = qprimeprime * sdot^2 + qprime * sdotdot
            double qdotdoti = qprimeprimei * sdot[i] * sdot[i] + qprimei * sdotdoti;
            // adjust sdot so that qdotdot is under the constraint
            double qdotdotmaxi = qdotdotmax(s[i]);
            if (Math.abs(qdotdoti) > qdotdotmaxi) {
                sdotdoti = (qdotdotmaxi - qprimeprimei * sdot[i] * sdot[i]) / qprimei;
                // integrate to find sdot
                // sdot = ds/dt
                // dt = ds/sdot
                double sdotnew = sdot[j] + sdotdoti * ds / sdot[i];
                if (Math.abs(sdotnew) < Math.abs(sdot[i]))
                    sdot[i] = sdotnew;
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
                // Integrate.
                // We have sdot(t) but we want to integrate over s to find t
                // The derivative of the inverse is the reciprocal
                // https://en.wikipedia.org/wiki/Inverse_function_rule
                // sdot(t) = 1/tprime(s)
                double ds = s[i] - s[i - 1];
                // tprime = dt/ds
                double tprimei = 1 / sdot[i];
                double dt1 = tprimei * ds;

                // trailing difference to get tprimeprime
                double tprimeprimei = (1 / sdot[i] - 1 / sdot[i - 1]) / ds;
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

}
