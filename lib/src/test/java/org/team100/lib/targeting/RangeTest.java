package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.targeting.Range.Solution;

public class RangeTest {
    private static final double DELTA = 0.001;

    @Test
    void testRange() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range r = new Range(d, 8, 50);
        Solution s = r.get(Math.PI / 4);
        assertEquals(2.826, s.range(), DELTA);
        assertEquals(1.010, s.tof(), DELTA);
    }

}
