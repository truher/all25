package org.team100.frc2025.Swerve.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Swerve.SemiAuto.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
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

public class EmbarkAndPreplace {
    /** Drive to the reef and go up. */
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

}
