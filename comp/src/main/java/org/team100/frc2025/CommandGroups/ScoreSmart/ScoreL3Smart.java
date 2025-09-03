package org.team100.frc2025.CommandGroups.ScoreSmart;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.CommandGroups.PrePlaceCoralL3;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Swerve.SemiAuto.Embark;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.Command;

public class ScoreL3Smart {
    public static Command get(
            LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            Supplier<ScoringPosition> height,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM,
            ReefPoint reefPoint) {

        Embark toReef = new Embark(
                logger, m_drive, heedRadiusM, controller, profile, height, reefPoint);
        PrePlaceCoralL3 prePlace = new PrePlaceCoralL3(wrist, elevator, tunnel, 23);

        return sequence(
                parallel(
                        toReef,
                        sequence(
                                wrist.readyUp().until(wrist::atGoal),
                                prePlace))
                        .until(() -> (toReef.isDone() && prePlace.isDone())),
                new PostDropCoralL3(wrist, elevator, 10));
    }
}
