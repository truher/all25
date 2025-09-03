package org.team100.frc2025.CommandGroups.ScoreSmart;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.CommandGroups.PrePlaceCoralL2;
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

public class ScoreL2Smart {
    public static Command get(
            LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            Supplier<ScoringPosition> height,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem drive,
            DoubleConsumer heedRadiusM,
            ReefPoint reefPoint) {

        Embark toReef = new Embark(
                logger, drive, heedRadiusM, controller, profile, height, reefPoint);
        PrePlaceCoralL2 prePlace = new PrePlaceCoralL2(wrist, elevator, 10.5);

        return sequence(
                parallel(
                        toReef,
                        sequence(
                                wrist.readyUp().until(wrist::atGoal),
                                prePlace))
                        .until(() -> (toReef.isDone() && prePlace.isDone())),
                parallel(
                        elevator.set(1.5),
                        wrist.set(0.70))
                        .until(elevator::atGoal));
    }
}
