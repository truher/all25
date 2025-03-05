package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

public class Spline1dTest {
    private static final double kDelta = 0.001;

    @Test
    void testVelocity() {
        // this is a sort of double u-turn
        Spline1d s = Spline1d.newSpline1d(0, 0, 1, 1, 0, 0);
        assertEquals(1, s.getVelocity(0), kDelta);
        for (double t = 0; t < 1; t+= 0.05) {
            Util.printf("%5.3f\n", s.getPosition(t));
        }
    }
    @Test
    void testZeroVelocity() {
        // this just never moves
        Spline1d s = Spline1d.newSpline1d(0, 0, 0, 0, 0, 0);
        assertEquals(0, s.getVelocity(0), kDelta);
        for (double t = 0; t < 1; t+= 0.05) {
            Util.printf("%5.3f\n", s.getPosition(t));
        }
    }
}
