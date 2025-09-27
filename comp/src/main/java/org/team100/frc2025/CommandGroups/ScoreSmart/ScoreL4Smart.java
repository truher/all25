package org.team100.frc2025.CommandGroups.ScoreSmart;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.Supplier;

import org.team100.frc2025.CalgamesArm.Placeholder;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.commands.drivetrain.DriveToPoseWithProfile;
import org.team100.lib.commands.r3.FollowTrajectory;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;


public class ScoreL4Smart {
    public static Command get(
            LoggerFactory logger,
            Placeholder placeholder,
            Manipulator manipulator,
            SwerveController controller,
            HolonomicProfile profile,
            SwerveDriveSubsystem m_drive,
            Supplier<Pose2d> goal) {
        DriveToPoseWithProfile toReef = new DriveToPoseWithProfile(
                logger, m_drive, controller, profile, goal);
        FollowTrajectory prePlace = placeholder.prePlaceL4(); //changed to follow trajectory
        return sequence( //this is differnt so we can like make it know about is done
                parallel(
                        toReef,
                        prePlace //
                ).until(() -> (toReef.isDone() && prePlace.isDone())),
                manipulator.centerEject()
                        .withTimeout(0.5),
                placeholder.stow());
    }
}
