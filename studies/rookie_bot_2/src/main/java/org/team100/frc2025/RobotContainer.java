package org.team100.frc2025;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import org.team100.lib.async.Async;
import org.team100.lib.async.AsyncFactory;
import org.team100.lib.examples.tank.DriveTank;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LevelPoller;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final TankDrive m_drive;
    private final Command m_auton;

    public RobotContainer(TimedRobot100 robot) throws IOException {
        final AsyncFactory asyncFactory = new AsyncFactory(robot);
        final Async async = asyncFactory.get();
        final Logging logging = Logging.instance();
        final LevelPoller poller = new LevelPoller(async, logging::setLevel, Level.COMP);
        System.out.printf("Using log level %s\n", poller.getLevel().name());
        System.out.println("Do not use TRACE in comp, with NT logging, it will overrun");

        final LoggerFactory logger = logging.rootLogger;

        final DriverXboxControl driverControl = new DriverXboxControl(0);

        m_drive = TankFactory.make(logger, 20);
        m_drive.setDefaultCommand(new DriveTank(driverControl::velocity, m_drive));

        new Trigger(driverControl::x).whileTrue(
                m_drive.run(() -> m_drive.set(1.0, 0.0))
                        .withTimeout(1.0));

        m_auton = m_drive.run(() -> m_drive.set(1.0, 0.0))
                .withTimeout(1.0);
    }

    public void onInit() {
    }

    public void onTeleopInit() {
    }

    public void periodic() {
    }

    public void onAuto() {
    }

    public void close() {
    }

    private void whileTrue(BooleanSupplier condition, Command command) {
        new Trigger(condition).whileTrue(command);
    }

    public void scheduleAuton() {
        if (m_auton == null)
            return;
        m_auton.schedule();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
