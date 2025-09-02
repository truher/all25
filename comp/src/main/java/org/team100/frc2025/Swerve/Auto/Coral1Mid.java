package org.team100.frc2025.Swerve.Auto;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.ScoreSmart.PostDropCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Elevator.SetElevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Swerve.SemiAuto.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
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
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

public class Coral1Mid extends SequentialCommandGroup100 {
    public Coral1Mid(
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
        super(logger, "Coral1Mid");

        Embark embarkToH = new Embark(
                m_logger, drive, heedRadiusM, controller, profile, FieldSector.GH,
                ReefDestination.LEFT, () -> ScoringPosition.L4, ReefPoint.H);

        PrePlaceCoralL4 prePlaceCoralL4H = new PrePlaceCoralL4(wrist, elevator, tunnel, 47);

        addCommands(
                new ParallelCommandGroup100(m_logger, "embark1",
                        embarkToH,
                        new SequentialCommandGroup100(m_logger, "handoff then place",
                                new RunFunnelHandoff(m_logger, elevator, wrist, funnel, tunnel, grip)
                                        .withTimeout(0.5),
                                new ParallelCommandGroup100(m_logger, "ready to go up",
                                        new SetWrist(wrist, 0.4),
                                        new SetElevator(m_logger, elevator, 1)),
                                prePlaceCoralL4H))
                        .until(() -> (embarkToH.isDone() && prePlaceCoralL4H.isDone())),
                new PostDropCoralL4(wrist, elevator, 10));
    }
}
