package org.team100.lib.profile.roadrunner;


import static org.junit.jupiter.api.Assertions.assertEquals;

// passes uncommented
// import com.acmerobotics.roadrunner.profile.MotionState;

import org.junit.jupiter.api.Test;

public class MotionStateTest {
    private static final double DELTA = 0.001;

    @Test
    void testBasic() {
        MotionState s = new MotionState(1, 0, 0, 0);
        MotionState s1 = s.get(1);
        assertEquals(1, s1.x(), DELTA);
        assertEquals(0, s1.v(), DELTA);
    }

    @Test
    void testV() {
        MotionState s = new MotionState(0, 1, 0, 0);
        MotionState s1 = s.get(1);
        assertEquals(1, s1.x(), DELTA);
        assertEquals(1, s1.v(), DELTA);
    }

    @Test
    void testA() {
        MotionState s = new MotionState(0, 0, 1, 0);
        MotionState s1 = s.get(1);
        assertEquals(0.5, s1.x(), DELTA);
        assertEquals(1, s1.v(), DELTA);
        assertEquals(1, s1.a(), DELTA);

    }
}