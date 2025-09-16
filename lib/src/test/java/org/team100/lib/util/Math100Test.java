package org.team100.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

class Math100Test {
    private static final double DELTA = 0.001;

    @Test
    void testEpsilonEquals() {
        assertTrue(Math100.epsilonEquals(1, 1));
        assertFalse(Math100.epsilonEquals(1, 1.01));
    }

    @Test
    void testSolveQuadraticTwo() {
        List<Double> roots = Math100.solveQuadratic(1, 0, -1);
        assertEquals(2, roots.size());
        assertEquals(1, roots.get(0), DELTA);
        assertEquals(-1, roots.get(1), DELTA);
    }

    @Test
    void testSolveQuadraticOne() {
        List<Double> roots = Math100.solveQuadratic(1, 2, 1);
        assertEquals(1, roots.size());
        assertEquals(-1, roots.get(0), DELTA);
    }

    @Test
    void testSolveQuadraticZero() {
        List<Double> roots = Math100.solveQuadratic(1, 0, 1);
        assertEquals(0, roots.size());
    }







   

    @Test
    void testGetMinDistance() {
        double measurement = 4;
        double x = 0;
        double d = Math100.getMinDistance(measurement, x);
        assertEquals(2 * Math.PI, d, DELTA);
    }

}
