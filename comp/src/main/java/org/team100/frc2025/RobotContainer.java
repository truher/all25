package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.CalgamesArm.CalgamesViz;
import org.team100.frc2025.CalgamesArm.FollowJointProfiles;
import org.team100.frc2025.CalgamesArm.ManualCartesian;
import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.Climber.ClimberCommands;
import org.team100.frc2025.Climber.ClimberIntake;
import org.team100.frc2025.Climber.ClimberVisualization;
import org.team100.frc2025.CommandGroups.MoveToAlgaePosition;
import org.team100.frc2025.CommandGroups.ScoreSmart.ScoreCoralSmart;
import org.team100.frc2025.Swerve.ManualWithBargeAssist;
import org.team100.frc2025.Swerve.ManualWithProfiledReefLock;
import org.team100.frc2025.Swerve.Auto.Coral1Left;
import org.team100.frc2025.grip.Manipulator;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.coherence.Takt;
import org.team100.lib.commands.drivetrain.ResetPose;
import org.team100.lib.commands.drivetrain.SetRotation;
import org.team100.lib.commands.drivetrain.manual.DriveManuallySimple;
import org.team100.lib.config.Camera;
import org.team100.lib.config.Identity;
import org.team100.lib.controller.drivetrain.FullStateSwerveController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.controller.drivetrain.SwerveControllerFactory;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.PIDFeedback;
import org.team100.lib.examples.semiauto.FloorPickSequence;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.GyroFactory;
import org.team100.lib.hid.Buttons2025;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.hid.OperatorXboxControl;
import org.team100.lib.indicator.LEDIndicator;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.FreshSwerveEstimate;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SimulatedTagDetector;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveLocal;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.targeting.SimulatedTargetWriter;
import org.team100.lib.targeting.Targets;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Try to keep this container clean; if there's something you want to keep but
 * don't need right now, cut and paste it into {@link RobotContainerParkingLot}.
 */
public class RobotContainer {
    // for background on drive current limits:
    // https://v6.docs.ctr-electronics.com/en/stable/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html
    // https://www.chiefdelphi.com/t/the-brushless-era-needs-sensible-default-current-limits/461056/51
    // https://docs.google.com/document/d/10uXdmu62AFxyolmwtDY8_9UNnci7eVcev4Y64ZS0Aqk
    // https://github.com/frc1678/C2024-Public/blob/17e78272e65a6ce4f87c00a3514c79f787439ca1/src/main/java/com/team1678/frc2024/Constants.java#L195
    // 2/26/25: Joel updated the supply limit to 90A, see 1678 code above. This is
    // essentially unlimited, so you'll need to run some other kind of limiter (e.g.
    // acceleration) to keep from browning out.
    private static final double DRIVE_SUPPLY_LIMIT = 90;
    private static final double DRIVE_STATOR_LIMIT = 110;

    private final SwerveModuleCollection m_modules;
    private final Command m_auton;

    // SUBSYSTEMS
    final SwerveDriveSubsystem m_drive;
    final Climber m_climber;
    final ClimberIntake m_climberIntake;
    final LEDIndicator m_leds;

    final SwerveKinodynamics m_swerveKinodynamics;

    private final Runnable m_simulatedTagDetector;
    private final Runnable m_targetSimulator;
    private final Runnable m_combinedViz;
    private final Runnable m_climberViz;
    private final Targets m_targets;
    private final Manipulator m_manipulator;
    private final CalgamesMech m_mech;

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.TRACE);
        Util.printf("Using log level %s\n", poller.getLevel().name());
        Util.println("Do not use TRACE in comp, with NT logging, it will overrun");
        final LoggerFactory fieldLogger = logging.fieldLogger;
        final FieldLogger.Log fieldLog = new FieldLogger.Log(fieldLogger);

        final LoggerFactory logger = logging.rootLogger;
        final LoggerFactory driveLog = logger.name("Drive");
        final LoggerFactory comLog = logger.name("Commands");
        final LoggerFactory coralSequence = logger.name("Coral Sequence");
        final LoggerFactory autoSequence = logger.name("Auto Sequence");

        final TrajectoryVisualization viz = new TrajectoryVisualization(fieldLogger);

        // CONTROLS
        final DriverXboxControl driver = new DriverXboxControl(0);
        final OperatorXboxControl operator = new OperatorXboxControl(1);
        final Buttons2025 buttons = new Buttons2025(2);

        m_swerveKinodynamics = SwerveKinodynamicsFactory.get();
        if (Identity.instance.equals(Identity.COMP_BOT)) {
            m_climber = new Climber(logger, new CanId(13));
            m_climberIntake = new ClimberIntake(logger, new CanId(14));
        } else {
            m_climber = new Climber(logger, new CanId(18));
            m_climberIntake = new ClimberIntake(logger, new CanId(0));
        }

        m_manipulator = new Manipulator(logger);
        m_mech = new CalgamesMech(logger, 0.5, 0.343);

        m_combinedViz = new CalgamesViz(m_mech);
        m_climberViz = new ClimberVisualization(m_climber, m_climberIntake);

        m_modules = SwerveModuleCollection.get(
                driveLog,
                DRIVE_SUPPLY_LIMIT,
                DRIVE_STATOR_LIMIT,
                m_swerveKinodynamics);

        final Gyro gyro = GyroFactory.get(
                driveLog,
                m_swerveKinodynamics,
                m_modules);

        // Ignores the rotation derived from vision.
        final SwerveHistory history = new SwerveHistory(
                m_swerveKinodynamics,
                gyro.getYawNWU(),
                m_modules.positions(),
                Pose2d.kZero,
                Takt.get());

        final OdometryUpdater odometryUpdater = new OdometryUpdater(
                m_swerveKinodynamics, gyro, history, m_modules::positions);
        odometryUpdater.reset(Pose2d.kZero);
        final NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(history, odometryUpdater);

        final AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();

        final AprilTagRobotLocalizer localizer = new AprilTagRobotLocalizer(
                driveLog,
                layout,
                history,
                visionUpdater);

        m_targets = new Targets(driveLog, fieldLog, history);

        final FreshSwerveEstimate estimate = new FreshSwerveEstimate(
                localizer, odometryUpdater, history);

        final SwerveLocal swerveLocal = new SwerveLocal(
                driveLog,
                m_swerveKinodynamics,
                m_modules);

        final SwerveLimiter limiter = new SwerveLimiter(driveLog, m_swerveKinodynamics,
                RobotController::getBatteryVoltage);

        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                driveLog,
                m_swerveKinodynamics,
                odometryUpdater,
                estimate,
                swerveLocal,
                limiter);

        m_leds = new LEDIndicator(
                new RoboRioChannel(0),
                localizer::getPoseAgeSec,
                () -> (m_manipulator.hasCoral()
                        || (m_manipulator.hasCoralSideways()
                                && buttons.red2())),
                m_manipulator::hasAlgae,
                () -> (driver.rightTrigger()
                        || (driver.rightTrigger()
                                && driver.rightBumper())
                        || buttons.red2()),
                buttons::algae,
                buttons::red1,
                m_climberIntake::isIn);

        if (RobotBase.isReal()) {
            // Real robots get an empty simulated tag detector.
            m_simulatedTagDetector = () -> {
            };
            m_targetSimulator = () -> {
            };
        } else {
            // In simulation, we want the real simulated tag detector.
            SimulatedTagDetector sim = new SimulatedTagDetector(
                    List.of(
                            Camera.SWERVE_LEFT,
                            Camera.SWERVE_RIGHT,
                            Camera.FUNNEL,
                            Camera.CORAL_LEFT,
                            Camera.CORAL_RIGHT),
                    layout,
                    history);
            m_simulatedTagDetector = sim::periodic;
            // for now, one target, near the corner.
            SimulatedTargetWriter tsim = new SimulatedTargetWriter(
                    List.of(Camera.SWERVE_LEFT,
                            Camera.SWERVE_RIGHT,
                            Camera.FUNNEL,
                            Camera.CORAL_LEFT,
                            Camera.CORAL_RIGHT),
                    history,
                    new Translation2d[] {
                            FieldConstants.CoralMark.LEFT.value,
                            FieldConstants.CoralMark.CENTER.value,
                            FieldConstants.CoralMark.RIGHT.value });
            m_targetSimulator = tsim::update;
        }

        ////////////////////////////////////////////////////////////
        //
        // MECHANISM
        //

        // "fly" the joints manually
        whileTrue(operator::leftBumper,
                new ManualCartesian(operator::velocity, m_mech));
        // new ManualConfig(operatorControl::velocity, mech));

        ///////////////////////////
        //
        // DRIVETRAIN
        //

        // Reset pose estimator so the current gyro rotation corresponds to zero.
        onTrue(driver::back,
                new ResetPose(m_drive, new Pose2d()));

        // Reset pose estimator so the current gyro rotation corresponds to 180.
        onTrue(driver::start,
                new SetRotation(m_drive, Rotation2d.kPi));

        final Feedback100 thetaFeedback = new PIDFeedback(
                comLog, 3.2, 0, 0, true, 0.05, 1);

        // There are 3 modes:
        // * normal
        // * lock rotation to reef center
        // * barge-assist (slow when near barge)
        final Command driveDefault = new DriveManuallySimple(
                driver::velocity,
                localizer::setHeedRadiusM,
                m_drive,
                new ManualWithProfiledReefLock(
                        comLog, m_swerveKinodynamics, driver::leftTrigger,
                        thetaFeedback, m_drive),
                new ManualWithBargeAssist(
                        comLog, m_swerveKinodynamics, driver::pov,
                        thetaFeedback, m_drive),
                driver::leftBumper);

        /////////////////////////////////////////////////
        //
        // DEFAULT COMMANDS
        //

        m_drive.setDefaultCommand(driveDefault.withName("drive default"));
        // WARNING! This default command *MOVES IMMEDIATELY WHEN ENABLED*!
        m_mech.setDefaultCommand(m_mech.profileHomeAndThenRest().withName("mech default"));
        m_climber.setDefaultCommand(m_climber.stop().withName("climber default"));
        m_climberIntake.setDefaultCommand(m_climberIntake.stop().withName("climber intake default"));
        m_manipulator.setDefaultCommand(m_manipulator.stop().withName("manipulator default"));

        /////////////////////////////////////////////////
        //
        // AUTONOMOUS
        //

        final HolonomicProfile autoProfile = HolonomicProfile.currentLimitedExponential(1, 2, 4,
                m_swerveKinodynamics.getMaxAngleSpeedRad_S(), m_swerveKinodynamics.getMaxAngleAccelRad_S2(), 5);
        final FullStateSwerveController autoController = SwerveControllerFactory.auto2025LooseTolerance(autoSequence);

        m_auton = Coral1Left.get(logger, m_mech, m_manipulator,
                autoController, autoProfile, m_drive,
                localizer::setHeedRadiusM, m_swerveKinodynamics, viz);

        ////////////////////////////////////////////////////////////
        //
        // CORAL PICK
        //

        // At the same time, move the arm to the floor and spin the intake,
        // and go back home when the button is released, ending when complete.
        whileTrue(driver::rightTrigger,
                parallel(
                        m_mech.pickWithProfile(),
                        m_manipulator.centerIntake()))
                .onFalse(m_mech.profileHomeTerminal());

        // Move to coral ground pick location.
        whileTrue(driver::rightBumper,
                m_mech.pickWithProfile())
                .onFalse(m_mech.profileHomeTerminal());

        // Pick a game piece from the floor, based on camera input.
        whileTrue(() -> false,
                FloorPickSequence.get(
                        fieldLog, m_drive, m_targets,
                        SwerveControllerFactory.pick(driveLog), autoProfile)
                        .withName("Floor Pick"));

        whileTrue(() -> (driver.rightTrigger() && driver.rightBumper()),
                parallel(
                        m_mech.stationWithProfile(),
                        m_manipulator.centerIntake()));

        // Sideways intake for L1
        whileTrue(buttons::red2,
                sequence(
                        m_manipulator.sidewaysIntake()
                                .until(m_manipulator::hasCoralSideways),
                        m_manipulator.sidewaysHold()));

        ////////////////////////////////////////////////////////////
        //
        // CORAL SCORING
        //

        // Manual movement of arm, for testing.
        whileTrue(buttons::l1, m_mech.profileHomeToL1());
        // whileTrue(buttons::l2, mech.homeToL2()).onFalse(mech.l2ToHome());
        // whileTrue(buttons::l3, mech.homeToL3()).onFalse(mech.l3ToHome());
        // whileTrue(buttons::l4, mech.homeToL4()).onFalse(mech.l4ToHome());
        // whileTrue(driverControl::test, m_mech.homeToL4()).onFalse(m_mech.l4ToHome());

        final HolonomicProfile profile = HolonomicProfile.get(driveLog, m_swerveKinodynamics, 1, 0.5, 1, 0.2);
        final SwerveController holonomicController = SwerveControllerFactory.byIdentity(comLog);

        // Drive to a scoring location at the reef and score.
        whileTrue(driver::a,
                ScoreCoralSmart.get(
                        coralSequence, m_mech, m_manipulator,
                        holonomicController, profile, m_drive,
                        localizer::setHeedRadiusM, buttons::level, buttons::point));

        ////////////////////////////////////////////////////////////
        //
        // ALGAE
        //
        // Algae commands have two components: one button for manipulator,
        // one button for arm mechanism.

        // grab and hold algae, and then eject it when you let go of the button
        onTrue(buttons::algae,
                MoveToAlgaePosition.get(
                        m_mech, buttons::algaeLevel, buttons::algae));

        FollowJointProfiles homeGentle = m_mech.homeAlgae();
        whileTrue(driver::b, m_mech.algaePickGround()).onFalse(homeGentle.until(homeGentle::isDone));

        // Intake algae and puke it when you let go.
        whileTrue(buttons::barge,
                sequence(
                        m_manipulator.algaeIntake()
                                .until(m_manipulator::hasAlgae),
                        m_manipulator.algaeHold()) //
        ).onFalse(
                m_manipulator.algaeEject()
                        .withTimeout(0.5));

        // Move mech to processor
        whileTrue(buttons::red4,
                m_mech.processorWithProfile());

        // Move mech to barge
        whileTrue(buttons::red3,
                m_mech.homeToBarge()).onFalse(m_mech.bargeToHome());

        // whileTrue(driverControl::a, m_manipulator.run(m_manipulator::intakeCenter));
        // whileTrue(driverControl::b, m_manipulator.run(m_manipulator::ejectCenter));
        // whileTrue(driverControl::x, m_manipulator.run(m_manipulator::intakeCenter));

        ////////////////////////////////////////////////////////////
        //
        // CLIMB
        //

        // Extend, spin, wait for intake, and pull climber in and drive forward.
        whileTrue(buttons::red1,
                ClimberCommands.climbIntake(m_climber, m_climberIntake, m_mech));

        // Step 2, driver: Pull climber in and drive forward.
        onTrue(driver::y,
                ClimberCommands.climb(m_climber, m_drive, m_mech));

        // Between matches, operator: Reset the climber position.
        whileTrue(operator::rightBumper,
                m_climber.manual(operator::leftY));

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
                        startingBeeps(),
                        m_manipulator.centerIntake().withTimeout(1) //
                ).withName("test all movements") //
        );

        initStuff();
    }

    public void initStuff() {
        Util.println("\n*** PREWARM START");
        double startS = Takt.actual();

        // Exercise the trajectory planner.
        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(
                new Translation2d(),
                Rotation2d.kZero,
                Rotation2d.kZero));
        waypoints.add(new HolonomicPose2d(
                new Translation2d(1, 0),
                Rotation2d.kZero,
                Rotation2d.kZero));
        TrajectoryPlanner planner = new TrajectoryPlanner(
                new TimingConstraintFactory(m_swerveKinodynamics).medium());
        planner.restToRest(waypoints);

        // Exercise the drive motors.
        m_drive.driveInFieldCoords(new FieldRelativeVelocity(0, 0, 0));

        // Exercise some mechanism commands.
        Command c = m_mech.homeToL4();
        c.initialize();
        c.execute();

        c = m_mech.pickWithProfile();
        c.initialize();
        c.execute();

        m_mech.stop();

        c = m_manipulator.centerIntake();
        c.initialize();
        c.execute();

        m_manipulator.stopMotors();

        c = m_climber.goToIntakePosition();
        c.initialize();
        c.execute();

        m_climber.stopMotor();

        c = m_climberIntake.intake();
        c.initialize();
        c.execute();

        m_climberIntake.stopMotor();

        // Force full garbage collection.
        System.gc();

        double endS = Takt.actual();
        Util.printf("\n*** PREWARM END ET: %f\n", endS - startS);
    }

    public Command play(double freq) {
        return parallel(
                m_mech.play(freq),
                m_manipulator.play(freq));
    }

    /** Three beeps and one long beep. */
    public Command startingBeeps() {
        return sequence(
                play(880).withTimeout(0.5),
                play(0).withTimeout(0.5),
                play(880).withTimeout(0.5),
                play(0).withTimeout(0.5),
                play(880).withTimeout(0.5),
                play(0).withTimeout(0.5),
                play(1760).withTimeout(1.0),
                play(0).withTimeout(0.1));
    }

    public void onTeleop() {
        // TODO: remove
    }

    public void onInit() {
        m_drive.resetPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d(Math.PI)));
    }

    public void onAuto() {
        // TODO: remove
    }

    private Trigger whileTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).whileTrue(command);
    }

    private Trigger onTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).onTrue(command);
    }

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public void disabledPeriodic() {
        m_leds.disabledPeriodic();
    }

    public void enabledPeriodic() {
        m_leds.enabledPeriodic();
    }

    public void periodic() {
        // publish the simulated tag sightings.
        m_simulatedTagDetector.run();
        // publish simulated target sightings
        m_targetSimulator.run();
        // show the closest target on field2d
        m_targets.periodic();
        m_combinedViz.run();
        m_climberViz.run();
    }

    public void cancelAuton() {
        // TODO: remove
    }

    // this keeps the tests from conflicting via the use of simulated HAL ports.
    public void close() {
        m_modules.close();
        m_leds.close();
    }
}
