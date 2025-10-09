package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.state.GlobalSe2Velocity;

public class SwerveDeadbandTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testLargeInput() {
        // large input should be unaffected.
        GlobalSe2Velocity input = new GlobalSe2Velocity(1, 0, 0);
        SwerveDeadband deadband = new SwerveDeadband(logger);
        GlobalSe2Velocity result = deadband.apply(input);
        assertEquals(1, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }

    @Test
    void testSmallInput() {
        // small input should be ignored.
        // 5 mm/s is quite slow, maybe too slow?
        GlobalSe2Velocity input = new GlobalSe2Velocity(0.005, 0, 0);
        SwerveDeadband deadband = new SwerveDeadband(logger);
        GlobalSe2Velocity result = deadband.apply(input);
        assertEquals(0, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }
}
