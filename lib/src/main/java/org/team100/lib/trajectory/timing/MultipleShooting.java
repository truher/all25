package org.team100.lib.trajectory.timing;

import java.util.List;

import org.team100.lib.util.Math100;

/**
 * Experiment with a new implementation of the multiple shooting method.
 * 
 * There are constraints on V and A, as well as continuity of X and V.
 * 
 * @see https://en.wikipedia.org/wiki/Direct_multiple_shooting_method
 * @see https://en.wikipedia.org/wiki/Trajectory_optimization
 */
public class MultipleShooting {
    private final double m_maxV;
    private final double m_maxA;

    public MultipleShooting(double maxV, double maxA) {
        m_maxV = maxV;
        m_maxA = maxA;
    }

    public void solve(double[] x, double[] v, Double[] a) {
        int n = x.length;
        // estimate x_i from x_(i-1)
        double[] x_est = new double[n];
        double[] dt_est = new double[n - 1];
        for (int i = 1; i < n; ++i) {
            // forward integral
            double dx = x[i] - x[i - 1];
            double v0 = v[i - 1];
            // choose max accel in direction of x1 if there's no a estimate
            if (a[i - 1] == null) {
                a[i - 1] = Math.signum(dx) * m_maxA;
            }
            double a0 = a[i - 1];
            // maybe instead of solving for t (because it can fail), just guess?
            double A = 0.5 * a0;
            double B = v0;
            double C = -1.0 * dx;
            List<Double> soln = Math100.solveQuadratic(A, B, C);
            double dt = choose(soln);
            dt_est[i - 1] = dt;
            x_est[i] = x[i - 1] + v0 * dt + 0.5 * a0 * dt * dt;
        }
        // this contains v discontinuities
        simulate(x, v, a, dt_est);

    }

    private void simulate(double[] x, double[] v, Double[] a, double[] dt_est) {
        double DT = 0.01;
        double t0 = 0;
        System.out.println("t, a_t, v_t, x_t");
        for (int i = 0; i < x.length - 1; ++i) {
            double x0 = x[i];
            double v0 = v[i];
            double a0 = a[i];
            for (double t = 0; t < dt_est[i]; t += DT) {
                double v_t = v0 + a0 * t;
                double x_t = x0 + v0 * t + 0.5 * a0 * t * t;
                System.out.printf("%5.3f, %5.3f, %5.3f, %5.3f\n", t0 + t, a0, v_t, x_t);
            }
            t0 += dt_est[i];
        }
    }

    // choose smallest non-negative solution
    private double choose(List<Double> soln) {
        double x0 = Double.POSITIVE_INFINITY;
        for (double x : soln) {
            if (x >= 0 && x < x0)
                x0 = x;
        }
        if (Double.isFinite(x0))
            return x0;

        // no solution
        return 0;

        // throw new IllegalArgumentException();
    }

}
