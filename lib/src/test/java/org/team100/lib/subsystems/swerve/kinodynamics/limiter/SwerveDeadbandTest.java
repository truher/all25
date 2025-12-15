package org.team100.lib.subsystems.swerve.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

public class SwerveDeadbandTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testLargeInput() {
        // large input should be unaffected.
        VelocitySE2 input = new VelocitySE2(1, 0, 0);
        SwerveDeadband deadband = new SwerveDeadband(logger);
        VelocitySE2 result = deadband.apply(input);
        assertEquals(1, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }

    @Test
    void testSmallInput() {
        // small input should be ignored.
        // 5 mm/s is quite slow, maybe too slow?
        VelocitySE2 input = new VelocitySE2(0.005, 0, 0);
        SwerveDeadband deadband = new SwerveDeadband(logger);
        VelocitySE2 result = deadband.apply(input);
        assertEquals(0, result.x(), DELTA);
        assertEquals(0, result.y(), DELTA);
        assertEquals(0, result.theta(), DELTA);
    }
}
