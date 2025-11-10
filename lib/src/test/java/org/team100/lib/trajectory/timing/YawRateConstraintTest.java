package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;

class YawRateConstraintTest implements Timeless {
    private static final double DELTA = 0.001;
    // for testing, use the aboslute maximum. This shouldn't be used in a real
    // robot.
    private static final double YAW_RATE_SCALE = 1.0;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testNormal() {
        // towards +x, 1 rad/m, 1 m/s wheel -> 1 rad/s limit => 2.8 m/s (which violates
        // the linear constraint but it's ok)
        YawRateConstraint c = new YawRateConstraint(logger, SwerveKinodynamicsFactory.forTest(logger),
                YAW_RATE_SCALE);
        Pose2dWithMotion p = new Pose2dWithMotion(
                HolonomicPose2d.make(0, 0, 0, 0),
                1, // spatial, so rad/m
                0, 0);
        assertEquals(-8.485, c.getMinMaxAcceleration(p, 0).getMinAccel(), DELTA);
        assertEquals(8.485, c.getMinMaxAcceleration(p, 0).getMaxAccel(), DELTA);
        assertEquals(2.828, c.getMaxVelocity(p).getValue(), DELTA);
    }

    @Test
    void testVelocity2() {
        // towards +x, 1 rad/m, 2 rad/s limit => 2 m/s
        YawRateConstraint c = new YawRateConstraint(logger, SwerveKinodynamicsFactory.forTest2(logger),
                YAW_RATE_SCALE);
        Pose2dWithMotion p = new Pose2dWithMotion(
                HolonomicPose2d.make(0, 0, 0, 0), 1, // spatial, so rad/m
                0, 0);
        assertEquals(5.656, c.getMaxVelocity(p).getValue(), DELTA);
    }

    @Test
    void testAccel() {
        // we should impose an accel limit, now that the trajectory builder doesn't
        // force omega to zero at the start.
        YawRateConstraint c = new YawRateConstraint(logger, SwerveKinodynamicsFactory.forTest(logger),
                YAW_RATE_SCALE);
        // driving and spinning
        Pose2dWithMotion p = new Pose2dWithMotion(
                HolonomicPose2d.make(0, 0, 0, 0), 1,
                0, 0);
        // there is an accel limit.
        assertEquals(-8.485,
                c.getMinMaxAcceleration(p, 0).getMinAccel(), DELTA);
        assertEquals(8.485,
                c.getMinMaxAcceleration(p, 0).getMaxAccel(), DELTA);
    }

    @Test
    void testAccel2() {
        // towards +x, 1 rad/m, 2 rad/s limit => 2 m/s
        double scale = 0.1;
        YawRateConstraint c = new YawRateConstraint(logger, SwerveKinodynamicsFactory.forRealisticTest(logger),
                scale);
        Pose2dWithMotion p = new Pose2dWithMotion(
                HolonomicPose2d.make(0, 0, 0, 0),
                1, // spatial, so rad/m
                0, 0);
        // this number is still quite high even with a low scale.
        assertEquals(-16.971,
                c.getMinMaxAcceleration(p, 0).getMinAccel(), DELTA);
        assertEquals(16.971,
                c.getMinMaxAcceleration(p, 0).getMaxAccel(), DELTA);
    }
}
