package org.team100.frc2025.robot;

import java.io.IOException;

import org.team100.frc2025.CalgamesArm.CalgamesMech;
import org.team100.frc2025.CalgamesArm.CalgamesViz;
import org.team100.frc2025.Climber.Climber;
import org.team100.frc2025.Climber.ClimberIntake;
import org.team100.frc2025.Climber.ClimberVisualization;
import org.team100.frc2025.grip.Manipulator;
import org.team100.frc2025.indicator.LEDIndicator;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.GyroFactory;
import org.team100.lib.localization.AprilTagFieldLayoutWithCorrectOrientation;
import org.team100.lib.localization.AprilTagRobotLocalizer;
import org.team100.lib.localization.NudgingVisionUpdater;
import org.team100.lib.localization.OdometryUpdater;
import org.team100.lib.localization.SimulatedTagDetector;
import org.team100.lib.localization.SwerveHistory;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.motion.drivetrain.SwerveDriveFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.motion.drivetrain.module.SwerveModuleCollection;
import org.team100.lib.targeting.SimulatedTargetWriter;
import org.team100.lib.targeting.Targets;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.TimingConstraintFactory;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 {
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

    // LOGS
    final LoggerFactory m_logger;
    final FieldLogger.Log m_fieldLog;
    final LoggerFactory m_driveLog;
    private final RobotLog m_robotLog;

    private final SwerveModuleCollection m_modules;
    final AprilTagRobotLocalizer m_localizer;

    // SUBSYSTEMS
    final SwerveDriveSubsystem m_drive;
    final Climber m_climber;
    final ClimberIntake m_climberIntake;
    private final LEDIndicator m_leds;
    final Manipulator m_manipulator;
    final CalgamesMech m_mech;

    final SwerveKinodynamics m_swerveKinodynamics;

    final Beeper m_beeper;

    private final Machinery m_Machinery;
    private final AllAutons m_allAutons;
    private final Binder m_binder;

    private final Runnable m_simulatedTagDetector;
    private final Runnable m_targetSimulator;
    final TrajectoryVisualization m_trajectoryViz;
    private final Runnable m_combinedViz;
    private final Runnable m_climberViz;
    final Targets m_targets;
    final TrajectoryPlanner m_planner;

    public Robot() {
        // We want the CommandScheduler, not LiveWindow.
        enableLiveWindowInTest(false);

        // This is for setting up LaserCAN devices.
        // CanBridge.runTCP();

        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version);
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.instance.name());
        RobotController.setBrownoutVoltage(5.5);
        Banner.printBanner();

        // Log what the scheduler is doing. Use "withName()".
        SmartDashboard.putData(CommandScheduler.getInstance());

        final Async async = new AsyncFactory(this).get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.TRACE);
        System.out.printf("Using log level %s\n", poller.getLevel().name());
        System.out.println("Do not use TRACE in comp, with NT logging, it will overrun");
        final LoggerFactory fieldLogger = logging.fieldLogger;
        m_fieldLog = new FieldLogger.Log(fieldLogger);
        m_logger = logging.rootLogger;
        m_driveLog = m_logger.name("Drive");

        m_robotLog = new RobotLog(m_logger);

        m_swerveKinodynamics = SwerveKinodynamicsFactory.get();
        m_planner = new TrajectoryPlanner(
                new TimingConstraintFactory(m_swerveKinodynamics).medium());

        // TODO: move all the subsystems from here to Machinery.
        m_Machinery = new Machinery();

        ////////////////////////////////////////////////////////////
        //
        // SUBSYSTEMS
        //
        m_climber = new Climber(m_logger, new CanId(13));
        m_climberIntake = new ClimberIntake(m_logger, new CanId(14));
        m_manipulator = new Manipulator(m_logger);
        m_mech = new CalgamesMech(m_logger, 0.5, 0.343);

        ////////////////////////////////////////////////////////////
        //
        // VISUALIZATIONS
        //
        m_trajectoryViz = new TrajectoryVisualization(fieldLogger);
        m_combinedViz = new CalgamesViz(m_mech);
        m_climberViz = new ClimberVisualization(m_climber, m_climberIntake);

        ////////////////////////////////////////////////////////////
        //
        // POSE ESTIMATION
        //
        m_modules = SwerveModuleCollection.get(
                m_driveLog,
                DRIVE_SUPPLY_LIMIT,
                DRIVE_STATOR_LIMIT,
                m_swerveKinodynamics);
        final Gyro gyro = GyroFactory.get(
                m_driveLog,
                m_swerveKinodynamics,
                m_modules);
        final SwerveHistory history = new SwerveHistory(
                m_swerveKinodynamics,
                gyro.getYawNWU(),
                m_modules.positions(),
                Pose2d.kZero,
                Takt.get());
        final OdometryUpdater odometryUpdater = new OdometryUpdater(
                m_swerveKinodynamics, gyro, history, m_modules::positions);
        odometryUpdater.reset(Pose2d.kZero);
        final NudgingVisionUpdater visionUpdater = new NudgingVisionUpdater(
                history, odometryUpdater);

        ////////////////////////////////////////////////////////////
        //
        // CAMERA READERS
        //
        final AprilTagFieldLayoutWithCorrectOrientation layout = getLayout();

        m_localizer = new AprilTagRobotLocalizer(
                m_driveLog,
                layout,
                history,
                visionUpdater);
        m_targets = new Targets(m_driveLog, m_fieldLog, history);

        ////////////////////////////////////////////////////////////
        //
        // SIMULATED CAMERAS
        //
        m_simulatedTagDetector = SimulatedTagDetector.get(layout, history);
        m_targetSimulator = SimulatedTargetWriter.get(history);

        ////////////////////////////////////////////////////////////
        //
        // DRIVETRAIN
        //
        m_drive = SwerveDriveFactory.get(
                fieldLogger,
                m_driveLog,
                m_swerveKinodynamics,
                m_localizer,
                odometryUpdater,
                history,
                m_modules);
        m_drive.resetPose(new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d(Math.PI)));

        m_beeper = new Beeper(this);

        ////////////////////////////////////////////////////////////
        //
        // LED INDICATOR
        //
        m_leds = new LEDIndicator(
                m_localizer,
                m_manipulator,
                m_climberIntake);

        m_allAutons = new AllAutons(this);

        m_binder = new Binder(this);
        m_binder.bind();

        Prewarmer.init(this);
        System.out.printf("Total Logger Keys: %d\n", Logging.instance().keyCount());
    }

    @Override
    public void robotPeriodic() {
        // Advance the drumbeat.
        Takt.update();
        // Take all the measurements we can, as soon and quickly as possible.
        Cache.refresh();
        // Run one iteration of the command scheduler.
        CommandScheduler.getInstance().run();
        // publish the simulated tag sightings.
        m_simulatedTagDetector.run();
        // publish simulated target sightings
        m_targetSimulator.run();
        // show the closest target on field2d
        m_targets.periodic();
        m_leds.periodic();
        m_combinedViz.run();
        m_climberViz.run();
        m_robotLog.periodic();
        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            // StrUtil.warn("FLUSHING EVERY LOOP, DO NOT USE IN COMP");
            NetworkTableInstance.getDefault().flush();
        }
    }

    //////////////////////////////////////////////////////////////////////
    //
    // INITIALIZERS, DO NOT CHANGE THESE
    //

    @Override
    public void autonomousInit() {
        Command auton = m_allAutons.get();
        if (auton == null)
            return;
        auton.schedule();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void close() {
        super.close();
        // this keeps the tests from conflicting via the use of simulated HAL ports.
        m_modules.close();
        m_leds.close();
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // LEAVE ALL THESE EMPTY
    //

    @Override
    public void robotInit() {
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void testInit() {
    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testPeriodic() {
    }

    //////////////////////////////////////////////////////////////////

    /** Trap the IO exception. */
    private static AprilTagFieldLayoutWithCorrectOrientation getLayout() {
        try {
            return new AprilTagFieldLayoutWithCorrectOrientation();
        } catch (IOException e) {
            throw new IllegalStateException("Could not read Apriltag layout file", e);
        }
    }
}