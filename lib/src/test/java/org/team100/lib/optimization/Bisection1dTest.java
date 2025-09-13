package org.team100.lib.optimization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleUnaryOperator;

import org.junit.jupiter.api.Test;

public class Bisection1dTest {
    private static final double DELTA = 0.001;

    @Test
    void testFindRoot1dLinear() {
        double x_0 = -1;
        double f_0 = -1;
        double x_1 = 1;
        double f_1 = 1;
        int max_iterations = 1000;

        DoubleUnaryOperator func = (x) -> {
            return x;
        };

        // make sure we're passing the right bounds
        assertEquals(f_0, func.applyAsDouble(x_0), DELTA);
        assertEquals(f_1, func.applyAsDouble(x_1), DELTA);

        double s = Bisection1d.findRoot(func, x_0, f_0, x_1, f_1, DELTA, max_iterations);

        // between -1 and 1 the zero is half way
        assertEquals(0.5, s, DELTA);
    }

    @Test
    void testFindRoot1dLinear2() {
        double x_0 = -1;
        double f_0 = -2;
        double x_1 = 1;
        double f_1 = 0;
        int max_iterations = 1000;

        DoubleUnaryOperator func = (x) -> {
            return x - 1;
        };

        // make sure we're passing the right bounds
        assertEquals(f_0, func.applyAsDouble(x_0), DELTA);
        assertEquals(f_1, func.applyAsDouble(x_1), DELTA);

        double s = Bisection1d.findRoot(func, x_0, f_0, x_1, f_1, DELTA, max_iterations);

        assertEquals(1, s, DELTA);
    }
}
