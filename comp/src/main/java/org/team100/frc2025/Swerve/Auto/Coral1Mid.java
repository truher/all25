package org.team100.frc2025.Swerve.Auto;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.DoubleConsumer;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.commands.Done;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.field.FieldConstants.ReefPoint;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.wpilibj2.command.Command;

public class Coral1Mid {
    /** While driving to scoring tag, pay attention only to very close tags. */
    private static final double HEED_RADIUS_M = 3;

    public static Command get(
            LoggerFactory logger,
            CalgamesMech mech,
            Manipulator manipulator,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem drive,
            DoubleConsumer heedRadiusM,
            SwerveKinodynamics kinodynamics,
            TrajectoryVisualization viz) {

        DriveToPoseWithProfile toReef = new DriveToPoseWithProfile(
                logger, drive, controller, profile,
                () -> FieldConstants.makeGoal(ScoringLevel.L4, ReefPoint.H));

        Done prePlace = mech.homeToL4();
        return sequence(
                parallel(
                        runOnce(() -> heedRadiusM.accept(HEED_RADIUS_M)),
                        toReef,
                        prePlace //
                ).until(() -> (toReef.isDone() && prePlace.isDone())),
                manipulator.centerEject().withTimeout(0.5),
                mech.l4ToHome());
    }
}
