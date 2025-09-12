package org.team100.lib.commands.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ManualChassisSpeedsTest {
    private static final double DELTA = 0.001;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    @Test
    void testChassisSpeedsZero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        ManualChassisSpeeds manual = new ManualChassisSpeeds(logger, limits);
        DriverControl.Velocity input = new DriverControl.Velocity(0, 0, 0);
        ChassisSpeeds speeds = manual.apply(new SwerveModel(), input);
        assertEquals(0, speeds.vxMetersPerSecond, DELTA);
        assertEquals(0, speeds.vyMetersPerSecond, DELTA);
        assertEquals(0, speeds.omegaRadiansPerSecond, DELTA);
    }

    @Test
    void testChassisSpeedsNonzero() {
        SwerveKinodynamics limits = SwerveKinodynamicsFactory.forTest();
        assertEquals(1, limits.getMaxDriveVelocityM_S(), DELTA);
        assertEquals(2.828, limits.getMaxAngleSpeedRad_S(), DELTA);
        ManualChassisSpeeds manual = new ManualChassisSpeeds(logger, limits);
        // clipping to the unit circle
        DriverControl.Velocity input = new DriverControl.Velocity(1, 2, 3);
        ChassisSpeeds speeds = manual.apply(new SwerveModel(), input);
        assertEquals(0.447, speeds.vxMetersPerSecond, DELTA);
        assertEquals(0.894, speeds.vyMetersPerSecond, DELTA);
        assertEquals(2.828, speeds.omegaRadiansPerSecond, DELTA);
    }
}
