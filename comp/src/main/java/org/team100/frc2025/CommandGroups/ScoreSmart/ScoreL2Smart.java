package org.team100.frc2025.CommandGroups.ScoreSmart;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

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
import org.team100.lib.framework.ParallelCommandGroup100;
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
            SwerveDriveSubsystem drive,
            DoubleConsumer heedRadiusM,
            ReefPoint reefPoint) {
        super(logger, "ScoreL2Smart");

        Embark embarkCommand = new Embark(
                m_logger,
                drive,
                heedRadiusM,
                controller, profile,
                targetSector,
                destination,
                height,
                reefPoint);
        PrePlaceCoralL2 prePlaceCoralL2 = new PrePlaceCoralL2(wrist, elevator, 10.5);

        addCommands(
                new ParallelCommandGroup100(m_logger, "drive",
                        embarkCommand,
                        new SequentialCommandGroup100(m_logger, "out",
                                new SetWrist(wrist, 0.4),
                                prePlaceCoralL2))
                        .until(() -> (embarkCommand.isDone() && prePlaceCoralL2.isDone())),
                new ParallelDeadlineGroup100(m_logger, "down",
                        new SetElevator(m_logger, elevator, 1.5),
                        wrist.setPosition(0.70)));
    }
}
