package org.team100.lib.profile.roadrunner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;

public class DynamicProfileGeneratorTest {
    private static final double DELTA = 0.001;

    @Test
    void testGenerateMotionProfile() {
        MotionState start = new MotionState(0, 0, 0, 0);
        MotionState goal = new MotionState(5, 0, 0, 0);

        double resolution = 1;
        MotionProfile p = DynamicProfileGenerator.generateMotionProfile(
                start,
                goal,
                (s) -> 1,
                (s) -> 1,
                resolution);

        assertEquals(7, p.getSegments().size());
        assertEquals(0, p.get(0).x(), DELTA);
        assertEquals(0.5, p.get(1).x(), DELTA);
        assertEquals(1.5, p.get(2).x(), DELTA);
        assertEquals(2.5, p.get(3).x(), DELTA);
        assertEquals(3.5, p.get(4).x(), DELTA);
        assertEquals(4.5, p.get(5).x(), DELTA);
        assertEquals(5.0, p.get(6).x(), DELTA);

        assertEquals(6.0, p.duration(), DELTA);

        MotionState s0 = p.get(0);
        assertEquals(0, s0.v(), DELTA);

        MotionProfile p1 = p.append(p);
        assertEquals(12, p1.duration(), DELTA);

        MotionState s1 = p.get(1);
        assertEquals(1, s1.v(), DELTA);
    }

    @Test
    void testEvolve() {
        {
            // this is a bit nonsensical, this state would never move.
            MotionState s0 = new MotionState(0, 0, 0, 0);
            MotionState s1 = DynamicProfileGenerator.evolve(s0, 1);
            assertEquals(1, s1.x(), DELTA);
            assertEquals(0, s1.v(), DELTA);
            assertEquals(0, s1.a(), DELTA);
            assertEquals(0, s1.j(), DELTA);
        }
        {
            // nonzero v, zero a, end v is the same
            MotionState s0 = new MotionState(0, 1, 0, 0);
            MotionState s1 = DynamicProfileGenerator.evolve(s0, 1);
            assertEquals(1, s1.x(), DELTA);
            assertEquals(1, s1.v(), DELTA);
            assertEquals(0, s1.a(), DELTA);
            assertEquals(0, s1.j(), DELTA);
        }
        {
            // nonzero a, end v is more
            MotionState s0 = new MotionState(0, 0, 1, 0);
            MotionState s1 = DynamicProfileGenerator.evolve(s0, 1);
            assertEquals(1, s1.x(), DELTA);
            assertEquals(1.414, s1.v(), DELTA);
            assertEquals(1, s1.a(), DELTA);
            assertEquals(0, s1.j(), DELTA);
        }
        {
            // v is low, a is negative
            MotionState s0 = new MotionState(0, 0.1, -10, 0);
            assertThrows(IllegalArgumentException.class, () -> DynamicProfileGenerator.evolve(s0, 1));
        }
    }

    @Test
    void testComputeForward() {
        MotionState start = new MotionState(1, 0, 0, 0);
        int size = 3;
        double step = 0.3;
        List<MotionSpan> forwardStates = DynamicProfileGenerator.computeForward(
                start, (s) -> 2.0, (s) -> 6.0, step, size);
        assertEquals(4, forwardStates.size());
        // position shifted to start
        verify(forwardStates.get(0), 1, 0, 6, 0, 0.3); // full-length segment at max A
        verify(forwardStates.get(1), 1.3, 1.897, 6, 0, 0.033); // short segment to max V
        verify(forwardStates.get(2), 1.333, 2, 0, 0, 0.267);
        verify(forwardStates.get(3), 1.6, 2, 0, 0, 0.3);
    }

    @Test
    void testComputeBackwards() {
        MotionState goal = new MotionState(1.9, 0, 0, 0);
        int size = 3;
        double step = 0.3;
        List<MotionSpan> backwardStates = DynamicProfileGenerator.computeBackward(
                goal, (s) -> 2.0, (s) -> 6.0, step, size);
        assertEquals(4, backwardStates.size());
        verify(backwardStates.get(0), 1, 2, 0, 0, 0.3);
        verify(backwardStates.get(1), 1.3, 2, 0, 0, 0.267);
        verify(backwardStates.get(2), 1.567, 2, -6, 0, 0.033);
        verify(backwardStates.get(3), 1.6, 1.897, -6, 0, 0.3);
    }

    @Test
    void testIntersection() {
        MotionState s0 = new MotionState(0, 0, 1, 0);
        MotionState s1 = new MotionState(0, 0.5, 0.5, 0);
        double c = DynamicProfileGenerator.intersection(s0, s1);
        assertEquals(0.25, c, DELTA);
    }

    void verify(MotionSpan p, double x, double v, double a, double j, double dx) {
        assertEquals(x, p.start().x(), DELTA);
        assertEquals(v, p.start().v(), DELTA);
        assertEquals(a, p.start().a(), DELTA);
        assertEquals(j, p.start().j(), DELTA);
        assertEquals(dx, p.dx(), DELTA);

    }

}
