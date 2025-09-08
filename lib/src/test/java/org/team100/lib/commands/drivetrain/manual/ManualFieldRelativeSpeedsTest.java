package org.team100.lib.commands.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

class ManualFieldRelativeSpeedsTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testTwistZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(logger, limits);
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        SwerveModel s = new SwerveModel();
        FieldRelativeVelocity twist = manual.apply(s, input);
        assertEquals(0, twist.x(), DELTA);
        assertEquals(0, twist.y(), DELTA);
        assertEquals(0, twist.theta(), DELTA);
    }

    @Test
    void testTwistNonzero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualFieldRelativeSpeeds manual = new ManualFieldRelativeSpeeds(logger, limits);
        // these inputs are clipped
        DriverControl.Velocity input = new DriverControl.Velocity(1, 2, 3);
        SwerveModel s = new SwerveModel();
        FieldRelativeVelocity twist = manual.apply(s, input);
        assertEquals(0.447, twist.x(), DELTA);
        assertEquals(0.894, twist.y(), DELTA);
        assertEquals(2.828, twist.theta(), DELTA);
    }

}
