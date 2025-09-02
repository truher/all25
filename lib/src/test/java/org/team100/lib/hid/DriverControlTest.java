package org.team100.lib.hid;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class DriverControlTest {
    private static final double DELTA = 0.001;

    @Test
    void testClampTwist() {
        {
            // zero is no-op
            DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
            DriverControl.Velocity actual = input.clip(1);
            assertEquals(0, actual.x(), DELTA);
            assertEquals(0, actual.y(), DELTA);
            assertEquals(0, actual.theta(), DELTA);
        }
        {
            // clip to the unit circle.
            DriverControl.Velocity input = new DriverControl.Velocity(1, 1, 0);
            DriverControl.Velocity actual = input.clip(1);
            assertEquals(0.707, actual.x(), DELTA);
            assertEquals(0.707, actual.y(), DELTA);
        }
        {
            // leave the inside alone
            DriverControl.Velocity input = new DriverControl.Velocity(0.5, 0.5, 0);
            DriverControl.Velocity actual = input.clip(1);
            assertEquals(0.5, actual.x(), DELTA);
            assertEquals(0.5, actual.y(), DELTA);
        }
    }
}
