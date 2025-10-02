package org.team100.frc2025.setups;

import org.team100.frc2025.Swerve.ManualWithBargeAssist;
import org.team100.frc2025.Swerve.ManualWithProfiledReefLock;
import org.team100.lib.commands.drivetrain.manual.DriveManually;
import org.team100.lib.commands.drivetrain.manual.ManualChassisSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualFieldRelativeSpeeds;
import org.team100.lib.commands.drivetrain.manual.ManualWithFullStateHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithProfiledHeading;
import org.team100.lib.commands.drivetrain.manual.ManualWithTargetLock;
import org.team100.lib.commands.drivetrain.manual.SimpleManualModuleStates;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.ThirdControl;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * This is the set of manual driving modes we usually support,
 * but we're not using these in 2025, we're using DriveManuallySimple instead.
 */
public class DriveManuallySetup {

    public static void setup(
            LoggerFactory comLog,
            FieldLogger.Log fieldLog,
            DriverControl driverControl,
            AprilTagRobotLocalizer localizer,
            SwerveDriveSubsystem m_drive,
            SwerveKinodynamics m_swerveKinodynamics,
            Feedback100 thetaFeedback,
            ThirdControl buttons) {

        final DriveManually driveManually = new DriveManually(
                driverControl::velocity,
                localizer::setHeedRadiusM,
                m_drive);
        final LoggerFactory manLog = comLog.type(driveManually);

        driveManually.register("MODULE_STATE", false,
                new SimpleManualModuleStates(manLog, m_swerveKinodynamics));

        driveManually.register("ROBOT_RELATIVE_CHASSIS_SPEED", false,
                new ManualChassisSpeeds(manLog, m_swerveKinodynamics));

        driveManually.register("FIELD_RELATIVE_TWIST", false,
                new ManualFieldRelativeSpeeds(manLog, m_swerveKinodynamics));

        driveManually.register("SNAPS_PROFILED", true,
                new ManualWithProfiledHeading(
                        manLog,
                        m_swerveKinodynamics,
                        driverControl::desiredRotation,
                        thetaFeedback));

        driveManually.register("SNAPS_FULL_STATE", true,
                new ManualWithFullStateHeading(
                        manLog,
                        m_swerveKinodynamics,
                        driverControl::desiredRotation,
                        new double[] {
                                5,
                                0.35
                        }));

        /**
         * in reality, the target would come from some designator, e.g. buttons or
         * camera or whatever.
         */
        driveManually.register("LOCKED", false,
                new ManualWithTargetLock(
                        fieldLog,
                        manLog,
                        m_swerveKinodynamics,
                        () -> new Translation2d(6, 4),
                        thetaFeedback));

        driveManually.register("BARGE ASSIST", false,
                new ManualWithBargeAssist(
                        manLog,
                        m_swerveKinodynamics,
                        driverControl::desiredRotation,
                        thetaFeedback,
                        m_drive));

        driveManually.register("REEF LOCK", false,
                new ManualWithProfiledReefLock(
                        manLog,
                        m_swerveKinodynamics,
                        driverControl::useReefLock,
                        thetaFeedback,
                        m_drive));
    }
}
