package org.team100.frc2025.Swerve.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static org.team100.lib.config.ElevatorUtil.ScoringLevel.L4;
import static org.team100.lib.field.FieldConstants.ReefPoint.C;
import static org.team100.lib.field.FieldConstants.ReefPoint.D;
import static org.team100.lib.field.FieldConstants.ReefPoint.F;
import static org.team100.lib.field.FieldConstants.ReefPoint.I;
import static org.team100.lib.field.FieldConstants.ReefPoint.K;
import static org.team100.lib.field.FieldConstants.ReefPoint.L;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CalgamesArm.Placeholder;
import org.team100.frc2025.CommandGroups.PrePlaceCoralL4;
import org.team100.frc2025.CommandGroups.ScoreSmart.PostDropCoralL4;
import org.team100.frc2025.Elevator.Elevator;
import org.team100.frc2025.Funnel.Funnel;
import org.team100.frc2025.Wrist.AlgaeGrip;
import org.team100.frc2025.Wrist.CoralTunnel;
import org.team100.frc2025.Wrist.Wrist2;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.commands.drivetrain.DriveWithTrajectoryFunction;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.field.FieldConstants.CoralStation;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;

// it's a record to make it less verbose
public record Auton(LoggerFactory logger, Placeholder placeholder,
        Manipulator manipulator, Wrist2 wrist,
        Elevator elevator, Funnel funnel, CoralTunnel tunnel,
        AlgaeGrip grip, SwerveController controller, HolonomicProfile profile,
        SwerveDriveSubsystem drive,
        DoubleConsumer heedRadiusM, SwerveKinodynamics kinodynamics,
        TrajectoryVisualization viz) {

    /** While driving to scoring tag, pay attention only to very close tags. */
    private static final double HEED_RADIUS_M = 3;
    public Command left() {
        return sequence(
                embarkAndPreplace(L4, I),
                scoreAndReload(CoralStation.LEFT),
                embarkAndPreplace(L4, K),
                scoreAndReload(CoralStation.LEFT),
                embarkAndPreplace(L4, L),
                new PostDropCoralL4(wrist, elevator, 10)
                        .until(elevator::atGoal));
    }

    public Command right() {
        return sequence(
                embarkAndPreplace(L4, F),
                scoreAndReload(CoralStation.RIGHT),
                embarkAndPreplace(L4, D),
                scoreAndReload(CoralStation.RIGHT),
                embarkAndPreplace(L4, C),
                new PostDropCoralL4(wrist, elevator, 10)
                        .until(elevator::atGoal));
    }

    /** Drive to the reef and go up. */
    public Command embarkAndPreplace(ScoringLevel position, ReefPoint point) {
        DriveToPoseWithProfile toReef = new DriveToPoseWithProfile(
                logger, drive, controller, profile,
                () -> FieldConstants.makeGoal(position, point));
        PrePlaceCoralL4 prePlace = new PrePlaceCoralL4(wrist, elevator, tunnel, 47);
        return parallel(
                runOnce(() -> heedRadiusM.accept(HEED_RADIUS_M)),
                toReef,
                sequence(
                        wrist.readyUp().until(wrist::atGoal),
                        prePlace))
                .until(() -> (toReef.isDone() && prePlace.isDone()));
    }

    /** Score, drive to the station, and pause briefly. */
    public Command scoreAndReload(CoralStation station) {

        GoToCoralStation toStation = new GoToCoralStation(kinodynamics, station, 0.5);
        DriveWithTrajectoryFunction navigator = new DriveWithTrajectoryFunction(drive, controller, viz, toStation);
        return sequence(
                // first fire the coral at the peg
                manipulator.centerEject()
                        .withTimeout(0.5),
                parallel(
                        // then stow the arm
                        placeholder.stow(),
                        sequence(
                                // while the arm is still in motion
                                // wait for it to be low enough
                                waitUntil(placeholder::isSafeToDrive),
                                // and then drive to pick
                                navigator) //
                ).until(navigator::isDone),
                // then move the arm into the station and run the intake.
                parallel(
                        placeholder.station(),
                        manipulator.centerIntake() //
                ).until(manipulator::hasCoral) //
        );
    }
}
