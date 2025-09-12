package org.team100.lib.optimization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.DoubleBinaryOperator;

import org.junit.jupiter.api.Test;

public class RegulaFalsi2dTest {
    private static final double DELTA = 0.001;

    @Test
    void testFindRootLinear() {

        double x_0 = -1;
        double y_0 = 0;
        double f_0 = -1;
        double x_1 = 1;
        double y_1 = 0;
        double f_1 = 1;
        int max_iterations = 1000;

        DoubleBinaryOperator func = (x, y) -> {
            return x;
        };

        // make sure we're passing the right bounds
        assertEquals(f_0, func.applyAsDouble(x_0, y_0), DELTA);
        assertEquals(f_1, func.applyAsDouble(x_1, y_1), DELTA);

        double s = RegulaFalsi2d.findRoot(func, x_0, y_0, f_0, x_1, y_1, f_1, max_iterations);

        // between -1 and 1 the zero is half way
        assertEquals(0.5, s, DELTA);
    }

    @Test
    void testFindRootLinear2() {
        double x_0 = -1;
        double y_0 = 0;
        double f_0 = -1;
        double x_1 = 0.5;
        double y_1 = 0;
        double f_1 = 0.5;
        int max_iterations = 1000;

        DoubleBinaryOperator func = (x, y) -> {
            return x;
        };

        // make sure we're passing the right bounds
        assertEquals(f_0, func.applyAsDouble(x_0, y_0), DELTA);
        assertEquals(f_1, func.applyAsDouble(x_1, y_1), DELTA);

        double s = RegulaFalsi2d.findRoot(func, x_0, y_0, f_0, x_1, y_1, f_1, max_iterations);

        // between -1 and 0.5 the zero is at s = 2/3
        assertEquals(0.667, s, DELTA);
    }

    @Test
    void testFindRootLinear2d() {
        double x_0 = -1;
        double y_0 = -1;
        double f_0 = -2;
        double x_1 = 1;
        double y_1 = 1;
        double f_1 = 2;
        int max_iterations = 1000;

        DoubleBinaryOperator func = (x, y) -> {
            return x + y;
        };

        // make sure we're passing the right bounds
        assertEquals(f_0, func.applyAsDouble(x_0, y_0), DELTA);
        assertEquals(f_1, func.applyAsDouble(x_1, y_1), DELTA);

        double s = RegulaFalsi2d.findRoot(func, x_0, y_0, f_0, x_1, y_1, f_1, max_iterations);

        // the "zero" of x+y is a line, but this method only looks at
        // the line between x0,y0 and x1,y1, so it find the intersection at 0.
        assertEquals(0.5, s, DELTA);
    }

    @Test
    void testFindRootLog() {
        double x_0 = 0.1;
        double y_0 = 0;
        double f_0 = Math.log(0.1);
        double x_1 = 10;
        double y_1 = 0;
        double f_1 = Math.log(10);
        int max_iterations = 1000;

        DoubleBinaryOperator func = (x, y) -> {
            return Math.log(x);
        };

        // make sure we're passing the right bounds
        assertEquals(f_0, func.applyAsDouble(x_0, y_0), DELTA);
        assertEquals(f_1, func.applyAsDouble(x_1, y_1), DELTA);

        double s = RegulaFalsi2d.findRoot(func, x_0, y_0, f_0, x_1, y_1, f_1, max_iterations);

        assertEquals(0.091, s, DELTA);
    }

}
