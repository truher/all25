package org.team100.frc2025.Swerve.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.CommandGroups.RunFunnelHandoff;
import org.team100.frc2025.CommandGroups.ScoreSmart.PostDropCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Swerve.FieldConstants.ReefPoint;
import org.team100.frc2025.Swerve.SemiAuto.Embark;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;

public class Coral1Mid {
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

        Embark toReef = new Embark(
                logger, drive, heedRadiusM, controller, profile,
                () -> ScoringLevel.L4, ReefPoint.H);

        PrePlaceCoralL4 prePlace = new PrePlaceCoralL4(wrist, elevator, tunnel, 47);

        return sequence(
                parallel(
                        toReef,
                        sequence(
                                RunFunnelHandoff.get(logger, elevator, wrist, funnel, tunnel, grip).withTimeout(0.5),
                                parallel(
                                        wrist.readyUp(),
                                        elevator.set(1))
                                        .until(() -> wrist.atGoal() && elevator.atGoal()),
                                prePlace))
                        .until(() -> (toReef.isDone() && prePlace.isDone())),
                new PostDropCoralL4(wrist, elevator, 10)
                        .until(elevator::atGoal));
    }
}
