package org.team100.lib.math;

import java.util.function.Function;

import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.jni.EigenJNI;

/**
 * Newton's method finds a zero of a multivariate function.
 * 
 * For example, supply a function that describes the error in estimate and goal:
 * driving it to zero yields the x values to get the desired f.
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
    public Vector<X> solve(Vector<X> initial) {
        Vector<X> x = new Vector<>(initial.getStorage().copy());
        for (int i = 0; i < m_iterations; ++i) {
            Vector<Y> error = m_f.apply(x);
            if (within(error)) {
                return x;
            }
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

    /**
     * Single-sided Jacobian, faster.
     */
    public Vector<X> solve2(Vector<X> initialX) {
        long startTime = System.nanoTime();
        int iter = 0;
        try {
            Vector<X> x = new Vector<>(initialX.getStorage().copy());
            for (iter = 0; iter < m_iterations; ++iter) {

                Vector<Y> error = m_f.apply(x);
                if (within(error)) {
                    return x;
                }
                Matrix<Y, X> j = NumericalJacobian100.numericalJacobian2(m_xdim, m_ydim, m_f, x);

                // solve A x = B i.e. J dx = error
                Vector<X> dx = new Vector<>(j.solve(error));

                // this solver also works but it's not better.
                // Vector<X> dx = getDxWithQRDecomp(error, j);

                if (DEBUG)
                    System.out.printf("error: %s dx: %s\n",
                            Util.vecStr(error), Util.vecStr(dx));
                // Too-high dx results in oscillation.
                clamp(dx);
                update(x, dx);
                // Keep the x estimate within bounds.
                limit(x);
            }
            if (DEBUG)
                System.out.println("FAILED TO CONVERGE");
            throw new IllegalArgumentException(
                    String.format("failed to converge for inputs %s",
                            initialX));
            // return x;
        } finally {
            long finishTime = System.nanoTime();
            if (DEBUG)
                Util.printf("solve2 iterations: %d ET (ms): %6.3f\n",
                        iter, ((double) finishTime - startTime) / 1000000);
        }
    }

    /** A different solver */
    Vector<X> getDxWithQRDecomp(Vector<Y> error, Matrix<Y, X> j) {
        double[] A = j.getData();
        int Arows = j.getNumRows();
        int Acols = j.getNumCols();
        double[] B = error.getData();
        int Brows = error.getNumRows();
        int Bcols = error.getNumCols();
        // dst has same dimensions as B
        double[] dst = new double[B.length];
        // solve Ax=B
        EigenJNI.solveFullPivHouseholderQr(A, Arows, Acols, B, Brows, Bcols, dst);
        Vector<X> dx = new Vector<>(new Matrix<>(m_xdim, Nat.N1(), dst));
        return dx;
    }

    /**
     * Using dot instead of norm saves the sqrt.
     */
    private boolean within(Vector<Y> error) {
        double sqErr = error.dot(error);
        if (DEBUG)
            System.out.printf("sqErr %f tolSq %f\n", sqErr, m_toleranceSq);
        return sqErr < m_toleranceSq;
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
     * The "x" space is Euclidean, so using a simple sum is ok.
     */
    private void update(Vector<X> x, Vector<X> dx) {
        for (int i = 0; i < x.getNumRows(); ++i) {
            double newXi = x.get(i) - dx.get(i);
            x.set(i, 0, newXi);
        }
    }

    /**
     * Limit: dx = clamp(dx) using a fixed limit.
     * Mutates dx to save allocations.
     */
    private void clamp(Vector<X> dx) {
        for (int i = 0; i < dx.getNumRows(); ++i) {
            double dxI = dx.get(i);
            if (Math.abs(dxI) > m_dxLimit) {
                if (DEBUG)
                    System.out.println("clamped!");
            }
            double clampedDxI = MathUtil.clamp(dxI, -m_dxLimit, m_dxLimit);
            // System.out.printf("clamp %d %15.10f %15.10f\n", i, dxI, clampedDxI);
            dx.set(i, 0, clampedDxI);
        }
    }

    static <R extends Num> void print(Vector<R> v) {
        for (int i = 0; i < v.getNumRows(); ++i) {
            if (DEBUG)
                System.out.printf("%6.3f ", v.get(i));
        }
    }
}
