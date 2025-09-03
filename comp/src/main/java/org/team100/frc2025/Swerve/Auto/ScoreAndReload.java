package org.team100.frc2025.Swerve.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.ScoreSequence;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.FieldConstants.CoralStation;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ScoreAndReload {

    /** Score, drive to the station, and pause briefly. */
    public static Command get(

            LoggerFactory logger,
            Wrist2 wrist,
            Elevator elevator,
            Funnel funnel,
            CoralTunnel tunnel,
            AlgaeGrip grip,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem drive,
            DoubleConsumer heedRadiusM,
            SwerveKinodynamics kinodynamics,
            TrajectoryVisualization viz,
            CoralStation station) {
        GoToCoralStation toStation = new GoToCoralStation(
                logger, drive, controller, viz, kinodynamics, station, 0.5);
        return sequence(
                parallel(
                        ScoreSequence.get(wrist, elevator, 42, 10),
                        sequence(
                                Commands.waitUntil(() -> (elevator.isSafeToDrive() && wrist.isSafeToDrive())),
                                parallel(
                                        toStation,
                                        funnel.agitate(),
                                        tunnel.go())))
                        .until(toStation::isDone),
                RunFunnelHandoff.get(
                        logger, elevator, wrist, funnel, tunnel, grip)
                        .withTimeout(0.5));
    }

}
