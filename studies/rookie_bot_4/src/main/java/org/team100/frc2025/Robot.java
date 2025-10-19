package org.team100.frc2025;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.examples.tank.DriveTank;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.examples.tank.TankDriveFactory;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 {
    private final TankDrive m_drive;
    private final Command m_auton;

    public Robot() {
        Banner.printBanner();
        DriverStation.silenceJoystickConnectionWarning(true);
        SmartDashboard.putData(CommandScheduler.getInstance());
        Logging logging = Logging.instance();
        LoggerFactory fieldLogger = logging.fieldLogger;
        DriverXboxControl driverControl = new DriverXboxControl(0);
        LoggerFactory logger = logging.rootLogger;
        m_drive = TankDriveFactory.make(
                fieldLogger,
                logger,
                20, // supply current
                new CanId(3), // left
                new CanId(27), // right
                6.0, // gear ratio
                0.15); // wheel dia (m)
        m_drive.setDefaultCommand(
                new DriveTank(() -> -1.0 * driverControl.rightY(),
                        () -> -1.0 * driverControl.rightX(),
                        m_drive));
        m_auton = null;
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
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
    }
}