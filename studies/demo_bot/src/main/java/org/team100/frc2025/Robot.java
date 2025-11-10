package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.repeatingSequence;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.subsystems.shooter.DrumShooterFactory;
import org.team100.lib.subsystems.shooter.DualDrumShooter;
import org.team100.lib.subsystems.shooter.IndexerServo;
import org.team100.lib.subsystems.shooter.PivotDefault;
import org.team100.lib.subsystems.shooter.PivotSubsystem;
import org.team100.lib.subsystems.shooter.Shoot;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamicsFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.limiter.SwerveLimiter;
import org.team100.lib.subsystems.tank.TankDrive;
import org.team100.lib.subsystems.tank.TankDriveFactory;
import org.team100.lib.subsystems.tank.commands.TankManual;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot100 {
    private static final double FIVE_TO_ONE = 5.2307692308;
    private static final double GEAR_RATIO = FIVE_TO_ONE * FIVE_TO_ONE;
    private static final double WHEEL_DIAM = 0.098425;
    private static final double MAX_SPEED_M_S = 3.0;
    private static final double MAX_OMEGA_RAD_S = 3.0;

    private final TankDrive m_drive;
    private final Command m_auton;
    private final DualDrumShooter m_shooter;
    private final IndexerServo m_indexer;
    private final PivotSubsystem m_pivot;

    public Robot() {
        Banner.printBanner();
        DriverStation.silenceJoystickConnectionWarning(true);
        Experiments.instance.show();
        SmartDashboard.putData(CommandScheduler.getInstance());

        Logging logging = Logging.instance();
        LoggerFactory fieldLogger = logging.fieldLogger;
        LoggerFactory logger = logging.rootLogger;

        DriverXboxControl driverControl = new DriverXboxControl(0);

        m_drive = TankDriveFactory.make(
                fieldLogger,
                logger,
                20,
                new CanId(3),
                new CanId(2),
                0.4,
                GEAR_RATIO,
                WHEEL_DIAM);
        SwerveKinodynamics kinodynamics = SwerveKinodynamicsFactory.tank(logger);
        SwerveLimiter limiter = new SwerveLimiter(
                logger, kinodynamics, RobotController::getBatteryVoltage);
        m_drive.setDefaultCommand(new TankManual(
                logger, driverControl::rightY, driverControl::rightX,
                MAX_SPEED_M_S, MAX_OMEGA_RAD_S, limiter, m_drive));

        m_shooter = DrumShooterFactory.make(logger, 20);
        m_shooter.setDefaultCommand(m_shooter.run(m_shooter::stop));

        m_indexer = new IndexerServo(logger, 0);
        m_indexer.setDefaultCommand(m_indexer.run(m_indexer::stop));

        m_pivot = new PivotSubsystem(logger, 15);
        m_pivot.setDefaultCommand(new PivotDefault(driverControl::leftY, m_pivot));

        // this shows two ways to do the "shoot when spinning fast enough" thing.

        // a command class that contains the condition
        new Trigger(driverControl::a).whileTrue(new Shoot(m_shooter, m_indexer));

        // "fluent" command assembly.
        new Trigger(driverControl::y).whileTrue(
                parallel(
                        m_shooter.spin(10),
                        repeatingSequence(
                                waitUntil(m_shooter::atGoal),
                                m_indexer.feed().withTimeout(0.5))));

        // whileTrue(driverControl::fullCycle, new ShootOne(m_shooter, m_indexer));
        new Trigger(driverControl::x).whileTrue(m_shooter.spin(10));
        m_auton = null;
    }

    /**
     * robotPeriodic is called in the IterativeRobotBase.loopFunc, which is what the
     * TimedRobot runs in the main loop.
     * 
     * This is what should do all the work.
     */
    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        if (Experiments.instance.enabled(Experiment.FlushOften)) {
            NetworkTableInstance.getDefault().flush();
        }
    }

    @Override
    public void autonomousInit() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        m_pivot.setEncoderPosition(Math.PI / 2);
        // new ZeroPivot(m_pivot).schedule();
    }

}