package org.team100.frc2025.Swerve.Auto;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CommandGroups.PostDropAndReadyFunnel;
import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.ScoreSmart.PostDropCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Funnel.RunFunnel;
import org.team100.frc2025.Swerve.SemiAuto.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.RunCoralTunnel;
import org.team100.frc2025.Wrist.SetWrist;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.commands.drivetrain.FieldConstants.CoralStation;
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
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Commands;

public class Coral2AutoRightNewNew extends SequentialCommandGroup100 {

    public Coral2AutoRightNewNew(
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
        Embark embarkToI = new Embark(
                m_logger, drive, heedRadiusM, controller, profile, FieldSector.EF,
                ReefDestination.RIGHT, () -> ScoringPosition.L4, ReefPoint.F);
        PrePlaceCoralL4 prePlaceCoralL4I = new PrePlaceCoralL4(wrist, elevator, tunnel, 47);
        addCommands(
                new ParallelCommandGroup100(m_logger, "embark1",
                        embarkToI,
                        new SequentialCommandGroup100(m_logger, "handoff then place",
                                new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip).withTimeout(0.5),
                                new SetWrist(wrist, 0.4),
                                prePlaceCoralL4I))
                        .until(() -> (embarkToI.isDone() && prePlaceCoralL4I.isDone())));

        // Place preload and reload
        GoToCoralStation goToStation1stTime = new GoToCoralStation(
                m_logger, drive, controller, viz, kinodynamics, CoralStation.Right, 0.5);
        PostDropAndReadyFunnel postDropAndReadyFunnelFromI = new PostDropAndReadyFunnel(
                wrist, elevator, 10, (goToStation1stTime::isDone));
        addCommands(
                new ParallelDeadlineGroup100(m_logger, "drive away",
                        postDropAndReadyFunnelFromI,
                        new SequentialCommandGroup100(m_logger, "wait then drive away to pick up",
                                Commands.waitUntil(postDropAndReadyFunnelFromI::indicateReadyToLeave),
                                new ParallelDeadlineGroup100(m_logger, "Pick up",
                                        goToStation1stTime,
                                        new RunFunnel(funnel),
                                        new RunCoralTunnel(tunnel, 1)))));

        // Go to peg K and pre place
        Embark embarkToK = new Embark(
                m_logger, drive, heedRadiusM, controller, profile, FieldSector.CD,
                ReefDestination.RIGHT, () -> ScoringPosition.L4, ReefPoint.D);
        PrePlaceCoralL4 prePlaceCoralL4K = new PrePlaceCoralL4(wrist, elevator, tunnel, 47);
        addCommands(
                new ParallelCommandGroup100(m_logger, "embark1",
                        embarkToK,
                        new SequentialCommandGroup100(m_logger, "handoff then place",
                                new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip).withTimeout(0.5),
                                new SetWrist(wrist, 0.4),
                                prePlaceCoralL4K))
                        .until(() -> (embarkToK.isDone() && prePlaceCoralL4K.isDone())));

        // Place coral and reload
        GoToCoralStation goToStation2ndTime = new GoToCoralStation(
                m_logger, drive, controller, viz, kinodynamics, CoralStation.Right, 0.5);
        PostDropAndReadyFunnel postDropAndReadyFunnelFromK = new PostDropAndReadyFunnel(
                wrist, elevator, 10, goToStation2ndTime::isDone);
        addCommands(
                new ParallelDeadlineGroup100(m_logger, "drive away",
                        postDropAndReadyFunnelFromK,
                        new SequentialCommandGroup100(m_logger, "wait then drive away to pick up",
                                Commands.waitUntil(postDropAndReadyFunnelFromK::indicateReadyToLeave),
                                new ParallelDeadlineGroup100(m_logger, "Pick up",
                                        goToStation2ndTime,
                                        new RunFunnel(funnel),
                                        new RunCoralTunnel(tunnel, 1)))));

        // Go to peg L and pre place
        Embark embarkToL = new Embark(
                m_logger, drive, heedRadiusM, controller, profile, FieldSector.CD,
                ReefDestination.LEFT, () -> ScoringPosition.L4, ReefPoint.C);
        PrePlaceCoralL4 prePlaceCoralL4L = new PrePlaceCoralL4(wrist, elevator, tunnel, 47);
        addCommands(
                new ParallelCommandGroup100(m_logger, "embark1",
                        embarkToL,
                        new SequentialCommandGroup100(m_logger, "handoff then place",
                                new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip).withTimeout(0.5),
                                new SetWrist(wrist, 0.4),
                                prePlaceCoralL4L))
                        .until(() -> embarkToL.isDone() && prePlaceCoralL4L.isDone()),
                new PostDropCoralL4(wrist, elevator, 10));
    }
}
