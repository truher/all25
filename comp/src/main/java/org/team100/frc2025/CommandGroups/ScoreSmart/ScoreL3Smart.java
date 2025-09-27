package org.team100.frc2025.CommandGroups.ScoreSmart;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.Supplier;

import org.team100.frc2025.CalgamesArm.Placeholder;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreL3Smart {
    public static Command get(
            LoggerFactory logger,
            Placeholder placeholder,
            Wrist2 wrist,
            Elevator elevator,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            Supplier<Pose2d> goal) {
        DriveToPoseWithProfile toReef = new DriveToPoseWithProfile(logger, m_drive, controller, profile, goal);
        Command prePlace = placeholder.prePlaceL3();
        return sequence(
                parallel(
                        toReef,
                        prePlace)
                        .until(() -> (toReef.isDone() && placeholder.atL3())),
                new PostDropCoralL3(wrist, elevator, 10));
    }
}
