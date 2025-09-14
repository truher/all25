package org.team100.lib.examples.semiauto;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.DriveToTranslationWithFront;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.targeting.Targets;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Use a sequence of commands to drive nearby a target with the intake facing
 * it, and then sweep the intake over the target.
 */
public class FloorPickSequence {
    // landing point distance to target
    private static final double DISTANCE = 1.0;

    public static Command get(
            FieldLogger.Log fieldLog,
            SwerveDriveSubsystem drive,
            Targets targets,
            SwerveController controller,
            HolonomicProfile profile) {
        Supplier<Optional<Translation2d>> target = () -> targets.getClosestTranslation2d();
        Supplier<Optional<Translation2d>> runway = () -> {
            Optional<Translation2d> t = target.get();
            if (t.isEmpty())
                return t;
            Translation2d goal = t.get();
            Rotation2d course = goal.minus(drive.getPose().getTranslation()).getAngle();
            Translation2d landing = goal.minus(new Translation2d(DISTANCE, course));
            return Optional.of(landing);
        };
        
        DriveToTranslationWithFront toRunway = new DriveToTranslationWithFront(
                fieldLog, runway, drive, controller, profile);
        DriveToTranslationWithFront toTarget = new DriveToTranslationWithFront(
                fieldLog, target, drive, controller, profile);

        return sequence(
                toRunway.until(toRunway::isDone),
                toTarget.until(toTarget::isDone));
    }

}
