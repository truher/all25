package org.team100.frc2025.Swerve.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static org.team100.lib.commands.drivetrain.FieldConstants.FieldSector.CD;
import static org.team100.lib.commands.drivetrain.FieldConstants.FieldSector.EF;
import static org.team100.lib.commands.drivetrain.FieldConstants.FieldSector.IJ;
import static org.team100.lib.commands.drivetrain.FieldConstants.FieldSector.KL;
import static org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint.C;
import static org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint.D;
import static org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint.F;
import static org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint.I;
import static org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint.K;
import static org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint.L;
import static org.team100.lib.config.ElevatorUtil.ScoringPosition.L4;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.ScoreSequence;
import org.team100.frc2025.CommandGroups.ScoreSmart.PostDropCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Swerve.SemiAuto.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.FieldConstants.CoralStation;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// it's a record to make it less verbose
public record Auton(LoggerFactory logger, Wrist2 wrist, Elevator elevator, Funnel funnel, CoralTunnel tunnel,
        AlgaeGrip grip, SwerveController controller, HolonomicProfile profile, SwerveDriveSubsystem drive,
        DoubleConsumer heedRadiusM, SwerveKinodynamics kinodynamics, TrajectoryVisualization viz) {

    public Command left() {
        return sequence(
                embarkAndPreplace(IJ, ReefDestination.LEFT, L4, I),
                scoreAndReload(CoralStation.Left),
                embarkAndPreplace(KL, ReefDestination.LEFT, L4, K),
                scoreAndReload(CoralStation.Left),
                embarkAndPreplace(KL, ReefDestination.RIGHT, L4, L),
                new PostDropCoralL4(wrist, elevator, 10)
                        .until(elevator::atGoal));
    }

    public Command right() {
        return sequence(
                embarkAndPreplace(EF, ReefDestination.RIGHT, L4, F),
                scoreAndReload(CoralStation.Right),
                embarkAndPreplace(CD, ReefDestination.RIGHT, L4, D),
                scoreAndReload(CoralStation.Right),
                embarkAndPreplace(CD, ReefDestination.LEFT, L4, C),
                new PostDropCoralL4(wrist, elevator, 10)
                        .until(elevator::atGoal));
    }

    /** Drive to the reef and go up. */
    public Command embarkAndPreplace(
            FieldSector sector,
            ReefDestination destination,
            ScoringPosition position,
            ReefPoint point) {
        Embark toReef = new Embark(
                logger, drive, heedRadiusM, controller, profile, sector,
                destination, () -> position, point);
        PrePlaceCoralL4 prePlace = new PrePlaceCoralL4(wrist, elevator, tunnel, 47);
        return parallel(
                toReef,
                sequence(
                        RunFunnelHandoff.get(
                                logger, elevator, wrist, funnel, tunnel, grip)
                                .withTimeout(0.5),
                        wrist.readyUp().until(wrist::atGoal),
                        prePlace))
                .until(() -> (toReef.isDone() && prePlace.isDone()));
    }

    /** Score, drive to the station, and pause briefly. */
    public Command scoreAndReload(CoralStation station) {
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
