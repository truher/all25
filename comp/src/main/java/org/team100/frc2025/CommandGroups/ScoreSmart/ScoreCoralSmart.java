package org.team100.frc2025.CommandGroups.ScoreSmart;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.select;

import java.util.Map;
import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Swerve.FieldConstants;
import org.team100.frc2025.Swerve.FieldConstants.ReefPoint;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ScoreCoralSmart {
    /** While driving to scoring tag, pay attention only to very close tags. */
    private static final double HEED_RADIUS_M = 3;

    public static Command get(
            LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            CoralTunnel tunnel,
            Supplier<ScoringLevel> level,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            DoubleConsumer heedRadiusM,
            ReefPoint point) {
        Supplier<Pose2d> goal = () -> FieldConstants.makeGoal(level.get(), point);
        return parallel(
                runOnce(() -> heedRadiusM.accept(HEED_RADIUS_M)),
                select(
                        Map.ofEntries(
                                Map.entry(
                                        ScoringLevel.L4,
                                        ScoreL4Smart.get(logger, wrist, elevator, tunnel,
                                                controller, profile, m_drive, goal)),
                                Map.entry(
                                        ScoringLevel.L3,
                                        ScoreL3Smart.get(logger, wrist, elevator, tunnel,
                                                controller, profile, m_drive, goal)),
                                Map.entry(
                                        ScoringLevel.L2,
                                        ScoreL2Smart.get(logger, wrist, elevator, tunnel,
                                                controller, profile, m_drive, goal))),
                        level));
    }
}
