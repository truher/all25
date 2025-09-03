package org.team100.frc2025.Swerve.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

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
import org.team100.lib.framework.SequentialCommandGroup100;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Commands;

public class Coral2AutoLeftNewNew extends SequentialCommandGroup100 {

    public Coral2AutoLeftNewNew(
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
            TrajectoryVisualization viz) {
        super(logger, "Coral2Auto");

        // Go to peg I and pre place preload
        {
            addCommands(EmbarkAndPreplace.get(
                    logger, wrist, elevator, funnel, tunnel, grip, controller,
                    profile, drive, heedRadiusM, kinodynamics, viz,
                    FieldSector.IJ, ReefDestination.LEFT,
                    ScoringPosition.L4, ReefPoint.I));
        }

        // Place preload and reload
        {
            GoToCoralStation toStation = new GoToCoralStation(
                    m_logger, drive, controller, viz, kinodynamics, CoralStation.Left, 0.5);
            addCommands(
                    parallel(
                            ScoreSequence.get(wrist, elevator, 42, 10),
                            sequence(
                                    // wait for the elevator to be "low enough"
                                    Commands.waitUntil(() -> (elevator.isSafeToDrive() && wrist.isSafeToDrive())),
                                    // drive to the station while running the funnel/tunnel
                                    parallel(
                                            toStation,
                                            funnel.agitate(),
                                            tunnel.go())))
                            .until(toStation::isDone),
                    // pause and intake briefly
                    RunFunnelHandoff.get(m_logger, elevator, wrist, funnel, tunnel, grip).withTimeout(0.5));
        }

        // Go to peg K and pre place
        {
            Embark toReef = new Embark(
                    m_logger, drive, heedRadiusM, controller, profile, FieldSector.KL,
                    ReefDestination.LEFT, () -> ScoringPosition.L4, ReefPoint.K);
            PrePlaceCoralL4 prePlace = new PrePlaceCoralL4(wrist, elevator, tunnel, 47);
            addCommands(
                    parallel(
                            toReef,
                            sequence(
                                    RunFunnelHandoff.get(m_logger, elevator, wrist, funnel, tunnel, grip)
                                            .withTimeout(0.5),
                                    wrist.readyUp().until(wrist::atGoal),
                                    prePlace))
                            .until(() -> (toReef.isDone() && prePlace.isDone())));
        }

        // Place coral and reload
        {
            GoToCoralStationPastGlass toStation = new GoToCoralStationPastGlass(
                    m_logger, drive, controller, viz, kinodynamics, CoralStation.Left, 0.5);
            addCommands(
                    parallel(
                            ScoreSequence.get(wrist, elevator, 42, 10),
                            sequence(
                                    Commands.waitUntil(() -> (elevator.isSafeToDrive() && wrist.isSafeToDrive())),
                                    parallel(
                                            toStation,
                                            funnel.agitate(),
                                            tunnel.go()))
                                    .until(toStation::isDone))
                            .withTimeout(2.6));
        }

        // Go to peg L and pre place
        {
            Embark toReef = new Embark(
                    m_logger, drive, heedRadiusM, controller, profile,
                    FieldSector.KL, ReefDestination.RIGHT, () -> ScoringPosition.L4, ReefPoint.L);
            PrePlaceCoralL4 prePlace = new PrePlaceCoralL4(wrist, elevator, tunnel, 47);
            addCommands(
                    parallel(
                            toReef,
                            sequence(
                                    wrist.readyUp().until(wrist::atGoal),
                                    prePlace))
                            .until(() -> (toReef.isDone() && prePlace.isDone())),
                    new PostDropCoralL4(wrist, elevator, 10));
        }
    }
}
