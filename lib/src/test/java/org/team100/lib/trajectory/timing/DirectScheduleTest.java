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

}
