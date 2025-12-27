package org.team100.lib.subsystems.swerve.commands.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;

class ManualFieldRelativeSpeedsTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testTwistZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest(logger);
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(logger, limits);
        Velocity input = new Velocity(0, 0, 0);
        ModelSE2 s = new ModelSE2();
        VelocitySE2 twist = manual.apply(s, input);
        assertEquals(0, twist.x(), DELTA);
        assertEquals(0, twist.y(), DELTA);
        assertEquals(0, twist.theta(), DELTA);
    }

    @Test
    void testTwistNonzero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest(logger);
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(logger, limits);
        // these inputs are clipped
        Velocity input = new Velocity(1, 2, 3);
        ModelSE2 s = new ModelSE2();
        VelocitySE2 twist = manual.apply(s, input);
        assertEquals(0.447, twist.x(), DELTA);
        assertEquals(0.894, twist.y(), DELTA);
        assertEquals(2.828, twist.theta(), DELTA);
    }

}
