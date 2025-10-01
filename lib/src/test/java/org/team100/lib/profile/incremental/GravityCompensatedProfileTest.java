package org.team100.lib.profile.incremental;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

public class GravityCompensatedProfileTest {
    private static final boolean DEBUG = false;
    private static final double DELTA = 0.001;

    @Test
    void test0() {
        GravityCompensatedProfile p = new GravityCompensatedProfile(0.5, 1, 10);
        Control100 c = p.calculate(0.02, new Control100(0, 0), new Model100(0, 0));
        // already at goal
        assertEquals(0, c.a(), DELTA);
    }

    @Test
    void test1() {
        GravityCompensatedProfile p = new GravityCompensatedProfile(0.5, 1, 10);
        Control100 c = p.calculate(0.02,
                new Control100(Math.PI / 2, 0), new Model100(0, 0));
        // at 90, going up, 10 total, 4.9 for gravity, 5.1 for motion
        assertEquals(-5.1, c.a(), DELTA);
    }

    @Test
    void test2() {
        GravityCompensatedProfile p = new GravityCompensatedProfile(0.5, 1, 10);
        Control100 c = p.calculate(0.02,
                new Control100(Math.PI / 2, 0), new Model100(Math.PI, 0));
        // at 90, going down, 14.9 total, 4.9 for gravity, 10 for motion
        assertEquals(14.9, c.a(), DELTA);
    }

    @Test
    void test3() {
        GravityCompensatedProfile p = new GravityCompensatedProfile(0.5, 1, 10);
        Control100 c = p.calculate(0.02,
                new Control100(0, 0), new Model100(Math.PI, 0));
        // at 0, going down, no gravity influence.
        assertEquals(10, c.a(), DELTA);
    }

    @Test
    void testGoingDown() {
        GravityCompensatedProfile p = new GravityCompensatedProfile(0.5, 5, 10);
        Control100 i = new Control100(0, 0);
        Model100 g = new Model100(Math.PI / 2, 0);
        System.out.println("t, x, v, a");
        for (double t = 0; t < 1; t += 0.02) {
            i = p.calculate(0.02, i, g);
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f\n",
                        t, i.x(), i.v(), i.a());
        }
    }

    @Test
    void testGoingUp() {
        GravityCompensatedProfile p = new GravityCompensatedProfile(0.5, 5, 10);
        Control100 i = new Control100(Math.PI / 2, 0);
        Model100 g = new Model100(0, 0);
        System.out.println("t, x, v, a");
        for (double t = 0; t < 1; t += 0.02) {
            i = p.calculate(0.02, i, g);
            if (DEBUG)
                System.out.printf("%6.3f, %6.3f, %6.3f, %6.3f\n",
                        t, i.x(), i.v(), i.a());
        }
    }
}
