package org.team100.lib.subsystems.swerve.commands.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.Velocity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.state.ModelSE2;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ManualChassisSpeedsTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testChassisSpeedsZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest(logger);
        ManualChassisSpeeds manual = new ManualChassisSpeeds(logger, limits);
        Velocity input = new Velocity(0, 0, 0);
        ChassisSpeeds speeds = manual.apply(new ModelSE2(), input);
        assertEquals(0, speeds.vxMetersPerSecond, DELTA);
        assertEquals(0, speeds.vyMetersPerSecond, DELTA);
        assertEquals(0, speeds.omegaRadiansPerSecond, DELTA);
    }

    @Test
    void testChassisSpeedsNonzero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest(logger);
        assertEquals(1, limits.getMaxDriveVelocityM_S(), DELTA);
        assertEquals(2.828, limits.getMaxAngleSpeedRad_S(), DELTA);
        ManualChassisSpeeds manual = new ManualChassisSpeeds(logger, limits);
        // clipping to the unit circle
        Velocity input = new Velocity(1, 2, 3);
        ChassisSpeeds speeds = manual.apply(new ModelSE2(), input);
        assertEquals(0.447, speeds.vxMetersPerSecond, DELTA);
        assertEquals(0.894, speeds.vyMetersPerSecond, DELTA);
        assertEquals(2.828, speeds.omegaRadiansPerSecond, DELTA);
    }
}
