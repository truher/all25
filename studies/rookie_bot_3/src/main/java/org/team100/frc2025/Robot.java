package org.team100.frc2025;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Identity;
import org.team100.lib.examples.tank.DriveTank;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot100 {

    private final TankDrive m_drive;
    private final Command m_auton;

    public Robot() {
        System.out.printf("Serial Number %s\n",  RobotController.getSerialNumber());
        banner();
        SmartDashboard.putData(CommandScheduler.getInstance());
        Logging logging = Logging.instance();
        logging.setLevel(Level.TRACE);
        LoggerFactory fieldLogger = logging.fieldLogger;
        DriverXboxControl driverControl = new DriverXboxControl(0);
        LoggerFactory logger = logging.rootLogger;
        m_drive = TankFactory.make(fieldLogger, logger, 80);
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

    private void banner() {
        StringBuilder b = new StringBuilder();
        b.append("\n");
        b.append("..########.########....###....##.....##.......##.....#####.....#####....\n");
        b.append(".....##....##.........##.##...###...###.....####....##...##...##...##...\n");
        b.append(".....##....##........##...##..####.####.......##...##.....##.##.....##..\n");
        b.append(".....##....######...##.....##.##.###.##.......##...##.....##.##.....##..\n");
        b.append(".....##....##.......#########.##.....##.......##...##.....##.##.....##..\n");
        b.append(".....##....##.......##.....##.##.....##.......##....##...##...##...##...\n");
        b.append(".....##....########.##.....##.##.....##.....######...#####.....#####....\n");
        b.append("\n");
        System.out.println(b.toString());
    }
}