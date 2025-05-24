package org.team100.lib.math;

import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;

/**
 * Newton's method finds a zero of a multivariate function; in this case, the
 * function is the error between the f and the desired value of f, so
 * driving it to zero yields the x values to get the desired f.
 * 
 * TODO: would it be better (i.e. faster) to make the error a scalar metric?
 * 
 * Uses the (estimated) Jacobian of the function to estimate the x intercept.
 * 
 * https://en.wikipedia.org/wiki/Newton%27s_method
 * https://hades.mech.northwestern.edu/images/7/7f/MR.pdf
 */
public class NewtonsMethod<X extends Num, Y extends Num> {
    private static final boolean DEBUG = false;
    private final Nat<X> m_xdim;
    private final Nat<Y> m_ydim;
    private final Function<Vector<X>, Vector<Y>> m_f;
    private final Vector<X> m_xMin;
    private final Vector<X> m_xMax;
    private final double m_toleranceSq;
    private final int m_iterations;
    /**
     * Max change in estimate per iteration, to avoid overreacting.
     */
    private final double m_dxLimit;

    public NewtonsMethod(
            Nat<X> xdim,
            Nat<Y> ydim,
            Function<Vector<X>, Vector<Y>> f,
            Vector<X> xMin,
            Vector<X> xMax,
            double tolerance,
            int iterations,
            double dxLimit) {
        m_xdim = xdim;
        m_ydim = ydim;
        m_f = f;
        m_xMin = xMin;
        m_xMax = xMax;
        m_toleranceSq = tolerance * tolerance;
        m_iterations = iterations;
        m_dxLimit = dxLimit;
    }

    /** Symmetric Jacobian, slower. */
    public Vector<X> solve(Vector<X> initial, Vector<Y> goal) {
        Vector<X> x = new Vector<>(initial.getStorage().copy());
        for (int i = 0; i < m_iterations; ++i) {
            Vector<Y> y = m_f.apply(x);
            dump(i, x, y);
            Vector<Y> error = goal.minus(y);
            if (within(error))
                return x;
            Matrix<Y, X> j = NumericalJacobian100.numericalJacobian(m_xdim, m_ydim, m_f, x);
            Vector<X> dx = new Vector<>(j.solve(error));
            // Too-high dx results in oscillation.
            clamp(dx);
            update(x, dx);
            // Keep the x estimate within bounds.
            limit(x);
        }
        return x;
    }

    /** Single-sided Jacobian, faster. */
    public Vector<X> solve2(Vector<X> initial, Vector<Y> goal) {
        Vector<X> x = new Vector<>(initial.getStorage().copy());
        for (int iter = 0; iter < m_iterations; ++iter) {
            Vector<Y> y = m_f.apply(x);
            dump(iter, x, y);
            Vector<Y> error = goal.minus(y);
            if (within(error)) {
                return x;
            }
            Matrix<Y, X> j = NumericalJacobian100.numericalJacobian2(m_xdim, m_ydim, m_f, x);
            Vector<X> dx = new Vector<>(j.solve(error));
            // Too-high dx results in oscillation.
            clamp(dx);
            update(x, dx);
            // Keep the x estimate within bounds.
            limit(x);
        }
        return x;
    }

    /**
     * Using dot instead of norm saves the sqrt.
     */
    private boolean within(Vector<Y> error) {
        return error.dot(error) < m_toleranceSq;
    }

    /**
     * Clamp the guess vector x to the per-dimension limits.
     * Mutates x to save allocations.
     */
    private void limit(Vector<X> x) {
        for (int i = 0; i < x.getNumRows(); ++i) {
            double xi = x.get(i);
            double xMin = m_xMin.get(i);
            double xMax = m_xMax.get(i);
            xi = MathUtil.clamp(xi, xMin, xMax);
            x.set(i, 0, xi);
        }
    }

    /**
     * Update: x = x + dx.
     * Mutates x to save allocations.
     */
    private void update(Vector<X> x, Vector<X> dx) {
        for (int i = 0; i < x.getNumRows(); ++i) {
            double newXi = x.get(i) + dx.get(i);
            x.set(i, 0, newXi);
        }
    }

    /**
     * Limit: dx = clamp(dx) using a fixed limit.
     * Mutates dx to save allocations.
     */
    private void clamp(Vector<X> dx) {
        for (int i = 0; i < dx.getNumRows(); ++i) {
            dx.set(i, 0, MathUtil.clamp(dx.get(i), -m_dxLimit, m_dxLimit));
        }
    }

    private void dump(int iter, Vector<X> x, Vector<Y> y) {
        if (!DEBUG)
            return;
        System.out.printf("%d: ", iter);
        System.out.printf("X [");
        print(x);
        System.out.printf("] Y [");
        print(y);
        System.out.println("]");
    }

    static <R extends Num> void print(Vector<R> v) {
        for (int i = 0; i < v.getNumRows(); ++i) {
            System.out.printf("%6.3f ", v.get(i));
        }
    }
}
