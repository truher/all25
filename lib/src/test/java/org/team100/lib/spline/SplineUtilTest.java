package org.team100.lib.spline;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

public class SplineUtilTest {
    private static final double kDelta = 0.001;

    @Test
    void testParabola() {
        Translation2d p1 = new Translation2d(-1, 1);
        Translation2d p2 = new Translation2d(0, 0);
        Translation2d p3 = new Translation2d(1, 1);
        double x = SplineUtil.fitParabola(p1, p2, p3);
        assertEquals(0, x, kDelta);
    }
}
