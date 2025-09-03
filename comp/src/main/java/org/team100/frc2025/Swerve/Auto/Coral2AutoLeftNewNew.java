package org.team100.frc2025.Swerve.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CommandGroups.ScoreSmart.PostDropCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
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

public class Coral2AutoLeftNewNew {

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
            TrajectoryVisualization viz) {
        return sequence(

                EmbarkAndPreplace.get(
                        logger, wrist, elevator, funnel, tunnel, grip, controller,
                        profile, drive, heedRadiusM, kinodynamics, viz,
                        FieldSector.IJ, ReefDestination.LEFT,
                        ScoringPosition.L4, ReefPoint.I),

                ScoreAndReload.get(logger, wrist, elevator, funnel, tunnel, grip, controller,
                        profile, drive, heedRadiusM, kinodynamics, viz, CoralStation.Left),

                EmbarkAndPreplace.get(
                        logger, wrist, elevator, funnel, tunnel, grip, controller,
                        profile, drive, heedRadiusM, kinodynamics, viz,
                        FieldSector.KL, ReefDestination.LEFT,
                        ScoringPosition.L4, ReefPoint.K),

                ScoreAndReload.get(logger, wrist, elevator, funnel, tunnel, grip, controller,
                        profile, drive, heedRadiusM, kinodynamics, viz, CoralStation.Left),

                EmbarkAndPreplace.get(
                        logger, wrist, elevator, funnel, tunnel, grip, controller,
                        profile, drive, heedRadiusM, kinodynamics, viz,
                        FieldSector.KL, ReefDestination.RIGHT,
                        ScoringPosition.L4, ReefPoint.L),

                new PostDropCoralL4(wrist, elevator, 10));
    }
}
