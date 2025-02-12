package org.team100.lib.controller.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.state.Model100;

class FullStateDriveControllerTest {
    private static final double kDelta = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testAtRest() {
        FullStateDriveController c = FullStateDriveController.getDefault(logger);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)));
        assertEquals(0, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertTrue(c.atReference());
    }

    @Test
    void testFar() {
        FullStateDriveController c = FullStateDriveController.getDefault(logger);
        assertFalse(c.atReference());
        // no velocity, no feedforward
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(1, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(1, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(4, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testFast() {
        FullStateDriveController c = FullStateDriveController.getDefault(logger);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(0, 1), // produces error = 1
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(0, 1), // produces FF = 1
                        new Model100(0, 0),
                        new Model100(0, 0)));
        // position err is zero but velocity error is 1 and feedforward is also 1 so dx
        // should be FF + K*e = 2
        assertEquals(1.25, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testOnTrack() {
        FullStateDriveController c = FullStateDriveController.getDefault(logger);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(-1, 0.5), // position + velocity error
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(-1, 0.5), // velocity reference
                        new Model100(0, 0),
                        new Model100(0, 0)));
        // position and velocity controls are opposite, so just cruise
        assertEquals(-3.375, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        assertEquals(0, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testAllAxes() {
        FullStateDriveController c = FullStateDriveController.getDefault(logger);
        assertFalse(c.atReference());
        // none of these have any velocity so there's no feedforward.
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(0, 0)),
                new SwerveModel(
                        new Model100(1, 0),
                        new Model100(2, 0),
                        new Model100(3, 0)),
                new SwerveModel(
                        new Model100(2, 0),
                        new Model100(4, 0),
                        new Model100(6, 0)));
        // 1m error so dx should be K*e = 1
        assertEquals(4, t.x(), kDelta);
        assertEquals(8, t.y(), kDelta);
        assertEquals(12, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

    @Test
    void testRotation() {
        FullStateDriveController c = FullStateDriveController.getDefault(logger);
        assertFalse(c.atReference());
        FieldRelativeVelocity t = c.calculate(
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(3, 0)),
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(-3, 0)),
                new SwerveModel(
                        new Model100(0, 0),
                        new Model100(0, 0),
                        new Model100(-3, 0)));
        assertEquals(0, t.x(), kDelta);
        assertEquals(0, t.y(), kDelta);
        // we want to rotate +
        assertEquals(1.133, t.theta(), kDelta);
        assertFalse(c.atReference());
    }

}
