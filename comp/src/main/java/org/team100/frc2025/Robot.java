package org.team100.frc2025;

import org.team100.frc2025.robot.AllAutons;
import org.team100.frc2025.robot.Binder;
import org.team100.frc2025.robot.Machinery;
import org.team100.frc2025.robot.Prewarmer;
import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Identity;
import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.logging.RobotLog;
import org.team100.lib.util.Banner;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 {

    private final RobotLog m_robotLog;
    private final Machinery m_machinery;
    private final AllAutons m_allAutons;
    private final Binder m_binder;

    public Robot() {
        // We want the CommandScheduler, not LiveWindow.
        enableLiveWindowInTest(false);

        // This is for setting up LaserCAN devices.
        // CanBridge.runTCP();

        System.out.printf("WPILib Version: %s\n", WPILibVersion.Version);
        System.out.printf("RoboRIO serial number: %s\n", RobotController.getSerialNumber());
        System.out.printf("Identity: %s\n", Identity.instance.name());
        RobotController.setBrownoutVoltage(5.5);
        DriverStation.silenceJoystickConnectionWarning(true);
        Banner.printBanner();

        // Log what the scheduler is doing. Use "withName()".
        SmartDashboard.putData(CommandScheduler.getInstance());

        final Async async = new AsyncFactory(this).get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.TRACE);
        System.out.printf("Using log level %s\n", poller.getLevel().name());
        System.out.println("Do not use TRACE in comp, with NT logging, it will overrun");

        LoggerFactory logger = logging.rootLogger;
        m_robotLog = new RobotLog(logger);

        m_machinery = new Machinery(logger);
        m_allAutons = new AllAutons(logger, m_machinery);
        m_binder = new Binder(m_machinery);
        m_binder.bind(logger);

        Prewarmer.init(m_machinery);
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
        m_machinery.periodic();
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
    public void close() {
        super.close();
        m_machinery.close();
    }

    ///////////////////////////////////////////////////////////////////////
    //
    // LEAVE ALL THESE EMPTY
    //

    @Override
    public void robotInit() {
    }

    @Override
    public void simulationInit() {
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

}