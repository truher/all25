package org.team100.frc2025.CommandGroups.ScoreSmart;

import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;

public class ScoreCoralSmart {
    public static Command get(LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            FieldSector targetSector,
            ReefDestination destination,
            Supplier<ScoringPosition> scoringPositionSupplier,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM,
            ReefPoint point) {
        return new SelectCommand<>(
                Map.of(
                        ScoringPosition.L4,
                        ScoreL4Smart.get(logger, wrist, elevator, tunnel,
                                targetSector, destination, scoringPositionSupplier, controller, profile,
                                m_drive, heedRadiusM, point),
                        ScoringPosition.L3,
                        ScoreL3Smart.get(logger, wrist, elevator, tunnel,
                                targetSector, destination, scoringPositionSupplier, controller, profile,
                                m_drive, heedRadiusM, point),
                        ScoringPosition.L2,
                        ScoreL2Smart.get(logger, wrist, elevator, tunnel,
                                targetSector, destination, scoringPositionSupplier, controller, profile,
                                m_drive, heedRadiusM, point)),
                scoringPositionSupplier);
    }
}
