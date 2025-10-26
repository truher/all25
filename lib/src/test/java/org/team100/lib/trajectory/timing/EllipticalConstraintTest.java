package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;

import edu.wpi.first.math.geometry.Pose2d;

public class EllipticalConstraintTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testVelocity() {
        EllipticalConstraint c = new EllipticalConstraint(logger, 2, 3, 4);
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 0, 0),
                0, 0);
        // moving purely in x, get the x number
        assertEquals(2, c.getMaxVelocity(state).getValue(), DELTA);
        state = new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(0, 1, 0),
                0, 0);
        // moving purely in y, get the y number
        assertEquals(3, c.getMaxVelocity(state).getValue(), DELTA);
        state = new Pose2dWithMotion(
                new Pose2d(),
                new MotionDirection(1, 1, 0),
                0, 0);
        // moving diagonally, get something in between
        assertEquals(2.353, c.getMaxVelocity(state).getValue(), DELTA);

    }

}
