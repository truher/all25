package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.CalgamesArm.CalgamesViz;
import org.team100.frc2025.CalgamesArm.ManualCartesian;
import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.Climber.ClimberCommands;
import org.team100.frc2025.Climber.ClimberIntake;
import org.team100.frc2025.Climber.ClimberVisualization;
import org.team100.frc2025.CommandGroups.GrabAndHoldAlgae;
import org.team100.frc2025.CommandGroups.ScoreSmart.ScoreCoralSmart;
import org.team100.frc2025.Swerve.ManualWithBargeAssist;
import org.team100.frc2025.Swerve.ManualWithProfiledReefLock;
import org.team100.frc2025.Swerve.Auto.Auton;
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
import org.team100.lib.examples.semiauto.FloorPickSetup;
import org.team100.lib.field.FieldConstants;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.GyroFactory;
import org.team100.lib.hid.DriverControl;
import org.team100.lib.hid.DriverControlProxy;
import org.team100.lib.hid.OperatorControl;
import org.team100.lib.hid.OperatorControlProxy;
import org.team100.lib.hid.ThirdControl;
import org.team100.lib.hid.ThirdControlProxy;
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
import org.team100.lib.util.Util;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
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

    private final ScheduledExecutorService m_initializer;

    private final Runnable m_simulatedTagDetector;
    private final Runnable m_targetSimulator;
    private final Runnable m_combinedViz;
    private final Runnable m_climberViz;
    private final Targets m_targets;
    private final Manipulator m_manipulator;

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
        final DriverControl driverControl = new DriverControlProxy(logger, async);
        final OperatorControl operatorControl = new OperatorControlProxy(async);
        final ThirdControl buttons = new ThirdControlProxy(async);

        m_swerveKinodynamics = SwerveKinodynamicsFactory.get();
        if (Identity.instance.equals(Identity.COMP_BOT)) {
            m_climber = new Climber(logger, 13); // should be correct
            m_climberIntake = new ClimberIntake(logger, 14); // should be correct
        } else {
            m_climber = new Climber(logger, 18);
            m_climberIntake = new ClimberIntake(logger, 0);
        }

        m_manipulator = new Manipulator(logger);
        CalgamesMech mech = new CalgamesMech(logger, 0.5, 0.343);

        m_combinedViz = new CalgamesViz(mech);
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

        // ignores the rotation derived from vision.
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

        SwerveLimiter limiter = new SwerveLimiter(driveLog, m_swerveKinodynamics, RobotController::getBatteryVoltage);

        m_drive = new SwerveDriveSubsystem(
                fieldLogger,
                driveLog,
                m_swerveKinodynamics,
                odometryUpdater,
                estimate,
                swerveLocal,
                limiter);

        m_leds = new LEDIndicator(0, localizer::getPoseAgeSec);

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

        ///////////////////////////
        //
        // DRIVE CONTROLLERS
        //

        final SwerveController holonomicController = SwerveControllerFactory.byIdentity(comLog);
        final Feedback100 thetaFeedback = new PIDFeedback(
                comLog, 3.2, 0, 0, true, 0.05, 1);

        // There are 4 modes:
        // * normal
        // * lock rotation to reef center
        // * lock rotation to nearest station
        // * and barge-assist
        // TODO: use a single selector in the control
        DriveManuallySimple driveDefault = new DriveManuallySimple(
                driverControl::velocity,
                localizer::setHeedRadiusM,
                m_drive,
                new ManualWithProfiledReefLock(
                        comLog, m_swerveKinodynamics, driverControl::useReefLock,
                        thetaFeedback, m_drive, buttons::red1),
                new ManualWithBargeAssist(
                        comLog, m_swerveKinodynamics, driverControl::desiredRotation,
                        thetaFeedback, m_drive),
                driverControl::driveWithBargeAssist);

        /////////////////////////////////////////////////
        //
        // DEFAULT COMMANDS

        m_drive.setDefaultCommand(driveDefault);

        // mech.setDefaultCommand(new HoldPosition(mech));
        // NOTE: this default command *MOVES IMMEDIATELY WHEN ENABLED*. WATCH OUT!
        mech.setDefaultCommand(mech.profileHomeEndless());

        m_climber.setDefaultCommand(
                m_climber.stop().withName("climber default"));
        m_climberIntake.setDefaultCommand(
                m_climberIntake.stop().withName("climber intake default"));

        m_manipulator.setDefaultCommand(
                m_manipulator.stop().withName("manipulator default"));

        // DRIVER BUTTONS
        final HolonomicProfile profile = HolonomicProfile.get(driveLog, m_swerveKinodynamics, 1, 0.5, 1, 0.2);

        final HolonomicProfile autoProfile = HolonomicProfile.currentLimitedExponential(3.5, 7, 20,
                m_swerveKinodynamics.getMaxAngleSpeedRad_S(), m_swerveKinodynamics.getMaxAngleAccelRad_S2(), 20);

        FullStateSwerveController autoController = SwerveControllerFactory.auto2025LooseTolerance(autoSequence);

        m_auton = new Auton(logger, mech, m_manipulator,
                autoController, autoProfile, m_drive,
                localizer::setHeedRadiusM, m_swerveKinodynamics, viz)
                .left();

        whileTrue(() -> false, // driverControl::test),
                new Auton(logger, mech, m_manipulator,
                        autoController, autoProfile, m_drive,
                        localizer::setHeedRadiusM, m_swerveKinodynamics, viz)
                        .right());

        // Driver/Operator Buttons
        onTrue(driverControl::resetRotation0, new ResetPose(m_drive, new Pose2d()));
        onTrue(driverControl::resetRotation180, new SetRotation(m_drive, Rotation2d.kPi));

        ////////////////////////////////////////////////////////////
        //
        // PICK
        //

        /**
         * At the same time, move the arm to the floor and spin the intake,
         * and go back home when the button is released, ending when complete.
         */
        whileTrue(driverControl::floorPick, //driver x
                parallel(
                        mech.pickWithProfile(),
                        m_manipulator.centerIntake()))
                .onFalse(mech.profileHomeTerminal());

        whileTrue(driverControl::stationPick,
                parallel(
                        mech.stationWithProfile(),
                        m_manipulator.centerIntake()));

        ////////////////////////////////////////////////////////////
        //
        // CLIMB
        //

        // Step 1, operator: Extend and spin.
        whileTrue(operatorControl::climbIntake, // mapped to operator x button
                ClimberCommands.intake(m_climber, m_climberIntake));

        // Step 2, driver: Pull climber in and drive forward.
        whileTrue(driverControl::climb, // mapped to driver y buttonTODO: Make sure this isnt double mapped
                ClimberCommands.climb(m_climber, m_drive));

        // Between matches, operator: Reset the climber position.
        whileTrue(operatorControl::activateManualClimb, // speed is operator get left Y, activated with op y button
                m_climber.manual(operatorControl::manualClimbSpeed));

        ////////////////////////////////////////////////////////////
        //
        // SCORING
        //

        // Manual movement of arm, for testing.
        whileTrue(buttons::l1, mech.homeToL1()).onFalse(mech.l1ToHome());
        whileTrue(buttons::l2, mech.homeToL2()).onFalse(mech.l2ToHome());
        whileTrue(buttons::l3, mech.homeToL3()).onFalse(mech.l3ToHome());
        whileTrue(buttons::l4, mech.homeToL4()).onFalse(mech.l4ToHome());

        // Driver controls "go to reef" mode, buttons supply level and point.
        whileTrue(driverControl::toReef,
                ScoreCoralSmart.get(
                        coralSequence, mech, m_manipulator,
                        holonomicController, profile, m_drive,
                        localizer::setHeedRadiusM, buttons::level, buttons::point));

        // grab and hold algae, and then eject it when you let go of the button
        whileTrue(buttons::algae,
                GrabAndHoldAlgae.get(
                        m_manipulator, mech, buttons::algaeLevel))
                .onFalse(m_manipulator.algaeEject().withTimeout(0.5));

        // these are all unbound
        whileTrue(buttons::red2, print("red2"));
        whileTrue(buttons::red3, print("red3"));
        whileTrue(buttons::red4, print("red4"));
        whileTrue(buttons::barge, print("barge"));

        whileTrue(driverControl::a, m_manipulator.run(m_manipulator::intakeCenter));
        whileTrue(driverControl::b, m_manipulator.run(m_manipulator::ejectCenter));
        // whileTrue(driverControl::x, m_manipulator.run(m_manipulator::intakeCenter));

        // "fly" the joints manually
        whileTrue(operatorControl::manual, // to go to manual, left bumper operator
                new ManualCartesian(operatorControl::velocity, mech));
        // new ManualConfig(operatorControl::velocity, mech));

        // this is for developing autopick.
        new FloorPickSetup(
                fieldLog, driverControl, m_drive, m_targets,
                SwerveControllerFactory.pick(driveLog), autoProfile);

        m_initializer = Executors.newSingleThreadScheduledExecutor();
        m_initializer.schedule(this::initStuff, 0, TimeUnit.SECONDS);

        // This really only does anything when we're sitting idle; when actually
        // running, the gc runs frequently without prodding.
        m_initializer.schedule(System::gc, 3, TimeUnit.SECONDS);
    }

    public void initStuff() {
        Util.println("\n******** Pre-initializing some expensive things ... ");
        double startS = Takt.actual();

        List<HolonomicPose2d> waypoints = new ArrayList<>();
        waypoints.add(new HolonomicPose2d(
                new Translation2d(),
                Rotation2d.kZero,
                Rotation2d.kZero));
        waypoints.add(new HolonomicPose2d(
                new Translation2d(1, 0),
                Rotation2d.kZero,
                Rotation2d.kZero));
        TrajectoryPlanner m_planner = new TrajectoryPlanner(
                new TimingConstraintFactory(m_swerveKinodynamics).medium());
        m_planner.restToRest(waypoints);
        m_drive.driveInFieldCoords(new FieldRelativeVelocity(0, 0, 0));
        System.gc();

        double endS = Takt.actual();
        Util.printf("\n... Initialization ET: %f\n", endS - startS);
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

    private Trigger whileTrue(BooleanSupplier condition1, BooleanSupplier condition2, Command command) {
        Trigger trigger1 = new Trigger(condition1);
        Trigger trigger2 = new Trigger(condition2);
        return trigger1.and(trigger2).whileTrue(command);
    }

    private Trigger onTrue(BooleanSupplier condition, Command command) {
        return new Trigger(condition).onTrue(command);
    }

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public void periodic() {
        // publish the simulated tag sightings.
        m_simulatedTagDetector.run();
        // publish simulated target sightings
        m_targetSimulator.run();
        // show the closest target on field2d
        m_targets.periodic();
        m_leds.periodic();
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
