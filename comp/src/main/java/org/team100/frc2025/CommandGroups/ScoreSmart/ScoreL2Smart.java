package org.team100.frc2025.CommandGroups.ScoreSmart;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.CommandGroups.DeadlineForEmbarkAndPrePlace;
import org.team100.frc2025.CommandGroups.PrePlaceCoralL2;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Swerve.SemiAuto.Embark;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.framework.ParallelDeadlineGroup100;
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

public class ScoreL2Smart extends SequentialCommandGroup100 {
    public ScoreL2Smart(
            LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> height,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM,
            ReefPoint reefPoint) {
        super(logger, "ScoreL2Smart");

        Embark embarkCommand = new Embark(
                logger,
                m_drive,
                heedRadiusM,
                controller, profile,
                targetSector,
                destination,
                height,
                reefPoint);
        PrePlaceCoralL2 prePlaceCoralL2 = new PrePlaceCoralL2(wrist, elevator, 10.5);

        addCommands(
                new ParallelDeadlineGroup100(logger, "drive",
                        new DeadlineForEmbarkAndPrePlace(
                                embarkCommand::isDone, prePlaceCoralL2::isDone),
                        embarkCommand,
                        new SequentialCommandGroup100(logger, "out",
                                new SetWrist(wrist, 0.4),
                                prePlaceCoralL2)),
                new ParallelDeadlineGroup100(logger, "down",
                        new SetElevator(logger, elevator, 1.5),
                        wrist.setPosition(0.70)));
    }
}
