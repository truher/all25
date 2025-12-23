package org.team100.lib.trajectory.timing;

import org.junit.jupiter.api.Test;
import org.team100.lib.trajectory.path.spline.SplineR1;

public class SchedulerTest {

    Scheduler100.Constraint constraint = new Scheduler100.Constraint() {
        @Override
        public double vmax(double q) {
            if (q > 0.45 && q < 0.55)
                return 0.5;
            return 1.0;
        }

        @Override
        public double amax(double q, double v) {
            if (q < 0.01) // soft start
                return (q * 500);
            return 5.0 - v;
        }

        @Override
        public double amin(double q, double v) {
            if (q > 0.99) // soft stop
                return (1 - q) * -500;
            return -10.0 + v;
        }
    };

    @Test
    void test0() {
        int n = 100;
        SplineR1 spline = SplineR1.get(0, 1, 0, 0, 0, 0);
        // these are the spline sample points; they can be
        // in arbitrary locations. s is immutable.
        double[] s = new double[n + 1];
        for (int i = 0; i < n + 1; ++i) {
            s[i] = (double) i / n;
        }

        // positions
        double[] q = new double[s.length];
        for (int i = 0; i < s.length; ++i) {
            q[i] = spline.getPosition(s[i]);
        }

        Scheduler100 scheduler = new Scheduler100(constraint);

        double[] dt = scheduler.schedule(q);

        dump(q, dt);
    }

    public static void dump(double[] q, double[] dt) {
        int n = q.length - 1;

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

}
