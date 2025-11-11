package org.team100.frc2025;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.mecanum.MecanumDrive100;
import org.team100.lib.subsystems.mecanum.MecanumDriveFactory;
import org.team100.lib.subsystems.mecanum.commands.ManualMecanum;
import org.team100.lib.subsystems.mecanum.kinematics.MecanumKinematics100.Slip;
import org.team100.lib.subsystems.shooter.DualDrumShooter;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;
import org.team100.lib.visualization.SpinnyVisualization;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot100 {
    private static final double MAX_SPEED_M_S = 3.0;
    private static final double MAX_SPEED_X_M_S = 3.5;
    private static final double MAX_SPEED_Y_M_S = 3.5;
    private static final double MAX_OMEGA_RAD_S = 5.0;
    private final DualDrumShooter m_shooter;
    private final SpinnyVisualization m_shooterViz;
    private final SpinnyVisualization m_indexerViz;
    private final MecanumDrive100 m_drive;
    private final Command m_auton;
    private final IndexerServo m_indexer;
    private final Autons m_autons;

    public Robot() {
        Banner.printBanner();
        DriverStation.silenceJoystickConnectionWarning(true);
        Experiments.instance.show();
        SmartDashboard.putData(CommandScheduler.getInstance());
        Logging logging = Logging.instance();
        LoggerFactory fieldLogger = logging.fieldLogger;
        DriverXboxControl driverControl = new DriverXboxControl(0);
        LoggerFactory logger = logging.rootLogger;
        m_drive = MecanumDriveFactory.make(
                fieldLogger,
                logger,
                45, // stator current limit (a)
                new CanId(60), // gyro
                new CanId(24), // front left
                new CanId(26), // front right
                new CanId(25), // rear left
                new CanId(27), // rear right
                0.533, // track width (m)
                0.406, // wheelbase (m)
                new Slip(1, 1.35, 1), // wheel slip corrections 1,1.4,1
                6.0, // gears
                0.15); // wheel dia (m)
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.mecanum(logger);
        SwerveLimiter limiter = new SwerveLimiter(
                logger,
                kinodynamics,
                RobotController::getBatteryVoltage);
        Command manual = new ManualMecanum(
                logger,
                driverControl::velocity,
                MAX_SPEED_X_M_S,
                MAX_SPEED_Y_M_S,
                MAX_OMEGA_RAD_S,
                limiter,
                m_drive);
        m_drive.setDefaultCommand(
                manual);

        // new Trigger(driverControl::x).whileTrue(
        // m_drive.run(() -> m_drive.setDutyCycle(1.0, 0.0))
        // .withTimeout(1.0));

        // // make a sort of S-shape, as an example of sequential commands.
        // new Trigger(driverControl::y).whileTrue(
        // sequence(
        // m_drive.driveWithVelocity(1.5, 2.5).withTimeout(1.0),
        // m_drive.driveWithVelocity(1.5, -2.5).withTimeout(1.0)));

        m_auton = m_drive.run(() -> m_drive.stop())
                .withTimeout(1.0);

        m_shooter = DrumShooterFactory.make(logger, 40);
        m_shooterViz = new SpinnyVisualization(m_shooter::get, "shooter", 5);
        m_shooter.setDefaultCommand(m_shooter.run(m_shooter::stop));

        m_indexer = new IndexerServo(logger, 0);
        m_indexerViz = new SpinnyVisualization(m_indexer::get, "indexer", 0.05);
        m_indexer.setDefaultCommand(m_indexer.run(m_indexer::stop));

        m_autons = new Autons(logger, fieldLogger, m_drive, m_indexer, m_shooter);

        new Trigger(driverControl::a).whileTrue(new Shoot(m_shooter, m_indexer, 9)); //////////////////////////////////////////
        new Trigger(driverControl::b).whileTrue(new Shoot(m_shooter, m_indexer, 8));
        new Trigger(driverControl::x).whileTrue(new Shoot(m_shooter, m_indexer, 10));
        new Trigger(driverControl::y).whileTrue(m_indexer.run(() -> m_indexer.set(-1)));
        new Trigger(driverControl::back).whileTrue(m_drive.resetPose());

    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_shooterViz.run();
        m_indexerViz.run();
        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void autonomousInit() {
        Command auton = m_autons.get().command();
        if (auton == null)
            return;
        auton.schedule();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
