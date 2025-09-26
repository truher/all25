package org.team100.frc2025.Climber;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import org.team100.frc2025.Swerve.DriveForwardSlowly;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberCommands {
    /**
     * Set arm position and start spinning.
     * Wait a second.
     * Command ends when spin velocity slows and the climber is in position
     */
    public static Command intake(Climber climber, ClimberIntake intake) {
        return parallel(
                climber.goToIntakePosition(),
                intake.intake())
                .withDeadline(
                        sequence(
                                waitSeconds(1),
                                waitUntil(
                                        () -> intake.isSlow()
                                                && climber.atGoal())))
                .withName("climb intake");
    }

    /**
     * Pull the climber in while driving slowly forward.
     */
    public static Command climb(Climber climber, SwerveDriveSubsystem drive) {
        return parallel(
                climber.goToClimbPosition(),
                new DriveForwardSlowly(drive))
                .withName("climb while driving");

    }
}
