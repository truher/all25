package org.team100.frc2025.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.function.BooleanSupplier;

import org.team100.frc2025.CalgamesArm.FollowJointProfiles;
import org.team100.frc2025.CalgamesArm.ManualCartesian;
import org.team100.frc2025.Climber.ClimberCommands;
import org.team100.frc2025.CommandGroups.MoveToAlgaePosition;
import org.team100.frc2025.CommandGroups.ScoreSmart.ScoreCoralSmart;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.examples.semiauto.FloorPickSequence;
import org.team100.lib.hid.Buttons2025;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.hid.OperatorXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.profile.HolonomicProfile;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Binds buttons to commands.
 */
public class Binder {

    private final Robot robot;
    public Binder(Robot robot) {
        this.robot = robot;
    }

    /** Call only this after all the member initialization in Robot is done! */
    public void bind(
            DriverXboxControl driver,
            OperatorXboxControl operator,
            Buttons2025 buttons) {

        ///////////////////////////
        //
        // DRIVETRAIN
        //
        // Reset pose estimator so the current gyro rotation corresponds to zero.
        onTrue(driver::back,
                new SetRotation(robot.m_drive, Rotation2d.kZero));

        // Reset pose estimator so the current gyro rotation corresponds to 180.
        onTrue(driver::start,
                new SetRotation(robot.m_drive, Rotation2d.kPi));

        ////////////////////////////////////////////////////////////
        //
        // MECHANISM
        //
        // "fly" the joints manually
        whileTrue(operator::leftBumper,
                new ManualCartesian(operator::velocity, robot.m_mech));
        // new ManualConfig(operatorControl::velocity, mech));

        ////////////////////////////////////////////////////////////
        //
        // CORAL PICK
        //

        // At the same time, move the arm to the floor and spin the intake,
        // and go back home when the button is released, ending when complete.
        whileTrue(driver::rightTrigger,
                parallel(
                        robot.m_mech.pickWithProfile(),
                        robot.m_manipulator.centerIntake()))
                .onFalse(robot.m_mech.profileHomeTerminal());

        // Move to coral ground pick location.
        whileTrue(driver::rightBumper,
                parallel(
                        robot.m_mech.pickWithProfile(),
                        robot.m_manipulator.centerIntake()))

                .onFalse(robot.m_mech.profileHomeTerminal());

        final HolonomicProfile coralPickProfile = HolonomicProfile.currentLimitedExponential(1, 2, 4,
                robot.m_swerveKinodynamics.getMaxAngleSpeedRad_S(), robot.m_swerveKinodynamics.getMaxAngleAccelRad_S2(),
                5);

        // Pick a game piece from the floor, based on camera input.
        whileTrue(operator::leftTrigger,
                parallel(
                        robot.m_mech.pickWithProfile(),
                        robot.m_manipulator.centerIntake(),

                        FloorPickSequence.get(
                                robot.m_fieldLog, robot.m_drive, robot.m_targets,
                                SwerveControllerFactory.pick(robot.m_driveLog), coralPickProfile)
                                .withName("Floor Pick"))
                        .until(robot.m_manipulator::hasCoral));

        FloorPickSequence.get(
                robot.m_fieldLog, robot.m_drive, robot.m_targets,
                SwerveControllerFactory.pick(robot.m_driveLog), coralPickProfile)
                .withName("Floor Pick")
                .until(robot.m_manipulator::hasCoral);

        // Sideways intake for L1
        whileTrue(buttons::red2,
                sequence(
                        robot.m_manipulator.sidewaysIntake()
                                .until(robot.m_manipulator::hasCoralSideways),
                        robot.m_manipulator.sidewaysHold()));

        ////////////////////////////////////////////////////////////
        //
        // CORAL SCORING
        //
        // Manual movement of arm, for testing.
        whileTrue(buttons::l1, robot.m_mech.profileHomeToL1());
        // whileTrue(buttons::l2, mech.homeToL2()).onFalse(mech.l2ToHome());
        // whileTrue(buttons::l3, mech.homeToL3()).onFalse(mech.l3ToHome());
        // whileTrue(buttons::l4, mech.homeToL4()).onFalse(mech.l4ToHome());
        // whileTrue(driverControl::test, m_mech.homeToL4()).onFalse(m_mech.l4ToHome());

        final LoggerFactory coralSequence = robot.m_logger.name("Coral Sequence");
        final HolonomicProfile profile = HolonomicProfile.get(coralSequence, robot.m_swerveKinodynamics, 1, 0.5, 1,
                0.2);
        final SwerveController holonomicController = SwerveControllerFactory.byIdentity(coralSequence);

        // Drive to a scoring location at the reef and score.
        whileTrue(driver::a,
                ScoreCoralSmart.get(
                        coralSequence, robot.m_mech, robot.m_manipulator,
                        holonomicController, profile, robot.m_drive,
                        robot.m_localizer::setHeedRadiusM, buttons::level, buttons::point));

        ////////////////////////////////////////////////////////////
        //
        // ALGAE
        //
        // Algae commands have two components: one button for manipulator,
        // one button for arm mechanism.

        // grab and hold algae, and then eject it when you let go of the button
        onTrue(buttons::algae,
                MoveToAlgaePosition.get(
                        robot.m_mech, buttons::algaeLevel, buttons::algae));

        FollowJointProfiles homeGentle = robot.m_mech.homeAlgae();
        whileTrue(driver::b, robot.m_mech.algaePickGround()).onFalse(homeGentle.until(homeGentle::isDone));

        // Intake algae and puke it when you let go.
        whileTrue(buttons::barge,
                sequence(
                        robot.m_manipulator.algaeIntake()
                                .until(robot.m_manipulator::hasAlgae),
                        robot.m_manipulator.algaeHold()) //
        ).onFalse(
                robot.m_manipulator.algaeEject()
                        .withTimeout(0.5));

        // Move mech to processor
        whileTrue(buttons::red4,
                robot.m_mech.processorWithProfile());

        // Move mech to barge
        whileTrue(buttons::red3,
                robot.m_mech.homeToBarge()).onFalse(robot.m_mech.bargeToHome());

        // whileTrue(driverControl::a, m_manipulator.run(m_manipulator::intakeCenter));
        // whileTrue(driverControl::b, m_manipulator.run(m_manipulator::ejectCenter));
        // whileTrue(driverControl::x, m_manipulator.run(m_manipulator::intakeCenter));

        ////////////////////////////////////////////////////////////
        //
        // CLIMB
        //
        // Extend, spin, wait for intake, and pull climber in and drive forward.
        whileTrue(buttons::red1,
                ClimberCommands.climbIntake(robot.m_climber, robot.m_climberIntake, robot.m_mech));

        // Step 2, driver: Pull climber in and drive forward.
        onTrue(driver::y,
                ClimberCommands.climb(robot.m_climber, robot.m_drive, robot.m_mech));

        // Between matches, operator: Reset the climber position.
        whileTrue(operator::rightBumper,
                robot.m_climber.manual(operator::leftY));

        ////////////////////////////////////////////////////////////
        //
        // TEST ALL MOVEMENTS
        //
        // For pre- and post-match testing.
        //
        // Enable "test" mode and press operator left bumper and driver right bumper.
        //
        // DANGER DANGER DANGER DANGER DANGER DANGER DANGER DANGER
        //
        // THIS WILL MOVE THE ROBOT VERY FAST!
        //
        // DO NOT RUN with the wheels on the floor!
        //
        // DO NOT RUN without tiedown clamps.
        //
        // DANGER DANGER DANGER DANGER DANGER DANGER DANGER DANGER
        //
        whileTrue(() -> (RobotState.isTest() && operator.leftBumper() && driver.rightBumper()),
                // for now, it just beeps and does one thing.
                sequence(
                        robot.m_beeper.startingBeeps(),
                        robot.m_manipulator.centerIntake().withTimeout(1) //
                ).withName("test all movements") //
        );

    }

    private static Trigger whileTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).whileTrue(command);
    }

    private static Trigger onTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).onTrue(command);
    }

}
