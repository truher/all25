package org.team100.lib.optimization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleUnaryOperator;

import org.junit.jupiter.api.Test;

public class NewtonsMethod1dTest {
    @Test
    void testLinear() {
        // linear function: f(x) = x + 1
        // exact answer in one iteration
        DoubleUnaryOperator f = x -> x + 1;
        double q0 = 0;
        double minQ = -10;
        double maxQ = 10;
        NewtonsMethod1d s = new NewtonsMethod1d(f, minQ, maxQ, 1e-3, 10, 1);
        // f(-1) = -1 + 1 = 0
        double x = s.solve(q0);
        assertEquals(-1, x, 1e-3);
    }

    @Test
    void testQuadratic1d() {
        // quadratic function: f(x) = x^2 - 2
        // good answer in 4 iterations
        DoubleUnaryOperator f = x -> (x * x - 2);
        double q0 = 0;
        double minQ = -10;
        double maxQ = 10;
        NewtonsMethod1d s = new NewtonsMethod1d(f, minQ, maxQ, 1e-3, 10, 1);
        double x = s.solve(q0);
        // f(1.414) = 2 - 2 = 0
        assertEquals(1.414, x, 1e-3);
    }

    @Test
    void testQuartic1d() {
        // quadratic function: f(x) = x^4 - 2
        DoubleUnaryOperator f = x -> (x * x * x * x - 2);
        double q0 = 0;
        double minQ = -10;
        double maxQ = 10;
        NewtonsMethod1d s = new NewtonsMethod1d(f, minQ, maxQ, 1e-3, 10, 1);
        double x = s.solve(q0);
        assertEquals(1.189, x, 1e-3);
    }
}
