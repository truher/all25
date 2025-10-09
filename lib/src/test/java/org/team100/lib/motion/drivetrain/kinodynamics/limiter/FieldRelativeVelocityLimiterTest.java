package org.team100.lib.motion.drivetrain.kinodynamics.limiter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.state.GlobalSe2Velocity;

public class FieldRelativeVelocityLimiterTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testDrive() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        assertEquals(5, k.getMaxDriveVelocityM_S(), DELTA);
        GlobalSe2Velocity s = new GlobalSe2Velocity(10, 0, 0);
        GlobalSe2Velocity i = limiter.proportional(s);
        assertEquals(5, i.x(), DELTA);
        assertEquals(0, i.y(), DELTA);
        assertEquals(0, i.theta(), DELTA);
    }

    @Test
    void testSpin() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        assertEquals(14.142, k.getMaxAngleSpeedRad_S(), DELTA);
        GlobalSe2Velocity target = new GlobalSe2Velocity(0, 0, -20);
        GlobalSe2Velocity i = limiter.proportional(target);
        assertEquals(0, i.x(), DELTA);
        assertEquals(0, i.y(), DELTA);
        assertEquals(-14.142, i.theta(), DELTA);

    }

    @Test
    void testMotionless() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(logger, limit);
        assertEquals(14.142, k.getMaxAngleSpeedRad_S(), DELTA);
        GlobalSe2Velocity t = new GlobalSe2Velocity(0, 0, 0);
        GlobalSe2Velocity i = l.preferRotation(t);
        assertEquals(0, i.x(), DELTA);
        assertEquals(0, i.y(), DELTA);
        assertEquals(0, i.theta(), DELTA);

    }

    @Test
    void testPreferRotation2() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(logger, limit);
        {
            // inside the envelope => no change
            GlobalSe2Velocity target = new GlobalSe2Velocity(1, 0, 1);
            GlobalSe2Velocity i = l.preferRotation(target);
            assertEquals(1, i.x(), DELTA);
            assertEquals(0, i.y(), DELTA);
            assertEquals(1, i.theta(), DELTA);
        }
        {
            // full v, half omega => half v
            GlobalSe2Velocity target = new GlobalSe2Velocity(5, 0, 7.05);
            GlobalSe2Velocity i = l.preferRotation(target);
            assertEquals(2.507, i.x(), DELTA);
            assertEquals(0, i.y(), DELTA);
            assertEquals(7.05, i.theta(), DELTA);
        }
        {
            // full v, full omega => zero v, sorry
            GlobalSe2Velocity target = new GlobalSe2Velocity(5, 0, 14.142);
            GlobalSe2Velocity i = l.preferRotation(target);
            assertEquals(0, i.x(), DELTA);
            assertEquals(0, i.y(), DELTA);
            assertEquals(14.142, i.theta(), DELTA);
        }
    }

    /** shouldn't allow any movement at 6v. */
    @Test
    void testBrownout() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 6);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        GlobalSe2Velocity target = new GlobalSe2Velocity(1, 0, 0);
        GlobalSe2Velocity limited = limiter.apply(target);
        assertEquals(0, limited.x(), DELTA);
        assertEquals(0, limited.y(), DELTA);
        assertEquals(0, limited.theta(), DELTA);
    }

    /** desaturation should keep the same instantaneous course. */
    @Test
    void testDesaturationCourseInvariant() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.forRealisticTest();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter l = new FieldRelativeVelocityLimiter(logger, limit);

        { // both motionless
            GlobalSe2Velocity target = new GlobalSe2Velocity(0, 0, 0);
            assertTrue(target.angle().isEmpty());
            GlobalSe2Velocity desaturated = l.apply(target);
            assertTrue(desaturated.angle().isEmpty());
        }
        { // translating ahead
            GlobalSe2Velocity target = new GlobalSe2Velocity(10, 0, 0);
            assertEquals(0, target.angle().get().getRadians(), DELTA);
            assertEquals(10, target.norm(), DELTA);
            GlobalSe2Velocity desaturated = l.apply(target);
            assertEquals(0, desaturated.angle().get().getRadians(), DELTA);
            assertEquals(5, desaturated.norm(), DELTA);
        }
        { // translating ahead and spinning
            GlobalSe2Velocity target = new GlobalSe2Velocity(10, 0, 10);
            assertEquals(0, target.angle().get().getRadians(), DELTA);
            assertEquals(10, target.norm(), DELTA);
            assertEquals(10, target.theta(), DELTA);
            GlobalSe2Velocity desaturated = l.apply(target);
            assertEquals(0, desaturated.angle().get().getRadians(), DELTA);
            assertEquals(3.694, desaturated.norm(), DELTA);
            assertEquals(3.694, desaturated.theta(), DELTA);
        }
        { // translating 45 to the left and spinning
            GlobalSe2Velocity target = new GlobalSe2Velocity(10 / Math.sqrt(2), 10 / Math.sqrt(2), 10);
            assertEquals(Math.PI / 4, target.angle().get().getRadians(), DELTA);
            assertEquals(10, target.norm(), DELTA);
            assertEquals(10, target.theta(), DELTA);
            GlobalSe2Velocity desaturated = l.apply(target);
            assertEquals(Math.PI / 4, desaturated.angle().get().getRadians(), DELTA);
            assertEquals(2.612, desaturated.x(), DELTA);
            assertEquals(2.612, desaturated.y(), DELTA);
            // slightly different due to drivetrain geometry
            assertEquals(3.693, desaturated.norm(), DELTA);
            assertEquals(3.693, desaturated.theta(), DELTA);
        }
    }

    @Test
    void driveAndSpinLimited() {
        SwerveKinodynamics k = SwerveKinodynamicsFactory.limiting();
        BatterySagSpeedLimit limit = new BatterySagSpeedLimit(logger, k, () -> 12);
        FieldRelativeVelocityLimiter limiter = new FieldRelativeVelocityLimiter(logger, limit);
        GlobalSe2Velocity target = new GlobalSe2Velocity(5, 0, 25);
        assertEquals(5, target.x(), DELTA);
        assertEquals(0, target.y(), DELTA);
        assertEquals(25, target.theta(), DELTA);
        // the spin is 5x the drive, as requested.
        // use the target as the previous setpoint
        GlobalSe2Velocity setpoint = limiter.apply(target);
        assertEquals(1.806, setpoint.x(), DELTA);
        assertEquals(0, setpoint.y(), DELTA);
        assertEquals(9.032, setpoint.theta(), DELTA);
    }

}
