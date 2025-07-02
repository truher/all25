package org.team100.lib.commands.drivetrain.manual;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.function.Supplier;

import org.junit.jupiter.api.Test;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.Fixtured;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.testing.Timeless;

class DriveManuallyTest extends Fixtured implements Timeless {
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    String desiredMode = null;
    DriverControl.Velocity desiredTwist = new DriverControl.Velocity(1, 0, 0);

    @Test
    void testSimple() {
        Supplier<DriverControl.Velocity> twistSupplier = () -> desiredTwist;
        SwerveDriveSubsystem drive = fixture.drive;
        fixture.collection.reset();
        stepTime();
        SwerveKinodynamics swerveKinodynamics = SwerveKinodynamicsFactory.forTest();

        DriveManually command = new DriveManually(
                twistSupplier,
                (x) -> {
                },
                drive);

        command.register("MODULE_STATE", false,
                new SimpleManualModuleStates(logger, swerveKinodynamics));

        command.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(logger, swerveKinodynamics));

        command.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(logger, swerveKinodynamics));

        command.overrideMode(() -> desiredMode);

        command.initialize();

        desiredMode = "MODULE_STATE";
        command.execute();

        stepTime();
        drive.periodic();
        assertEquals(1, drive.getChassisSpeeds().vx, 0.001);

        desiredMode = "ROBOT_RELATIVE_CHASSIS_SPEED";
        command.execute();

        desiredMode = "FIELD_RELATIVE_TWIST";
        command.execute();

        command.end(false);
    }

}
