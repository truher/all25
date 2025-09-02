package org.team100.frc2025.CommandGroups.ScoreSmart;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.CommandGroups.PrePlaceCoralL3;
import org.team100.frc2025.Elevator.Elevator;
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
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

public class ScoreL3Smart extends SequentialCommandGroup100 {
    public ScoreL3Smart(LoggerFactory logger,
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
        super(logger, "ScoreL4Smart");

        Embark embarkCommand = new Embark(
                m_logger, m_drive, heedRadiusM,
                controller, profile, targetSector,
                destination, height, reefPoint);
        PrePlaceCoralL3 prePlaceCoralL3 = new PrePlaceCoralL3(wrist, elevator, tunnel, 23);

        addCommands(
                new ParallelCommandGroup100(m_logger, "drive",
                        embarkCommand,
                        new SequentialCommandGroup100(m_logger, "out",
                                new SetWrist(wrist, 0.4),
                                prePlaceCoralL3))
                        .until(() -> (embarkCommand.isDone() && prePlaceCoralL3.isDone())),
                new PostDropCoralL3(wrist, elevator, 10));
    }
}
