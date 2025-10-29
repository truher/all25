package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.examples.mecanum.MecanumDrive;
import org.team100.lib.examples.mecanum.MecanumDriveFactory;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot100 {
    private static final double MAX_SPEED_M_S = 3.0;
    private static final double MAX_OMEGA_RAD_S = 3.0;

    private final MecanumDrive m_drive;
    private final Autons m_autons;

    public Robot() {
        Banner.printBanner();
        DriverStation.silenceJoystickConnectionWarning(true);
        SmartDashboard.putData(CommandScheduler.getInstance());
        Logging logging = Logging.instance();
        LoggerFactory fieldLogger = logging.fieldLogger;
        DriverXboxControl driverControl = new DriverXboxControl(0);
        LoggerFactory logger = logging.rootLogger;
        m_drive = MecanumDriveFactory.make(
                fieldLogger,
                logger,
                5, // supply limit -- current, in amps
                null, // gyro
                new CanId(2), // front left
                new CanId(1), // front right
                new CanId(3), // rear left
                new CanId(4), // rear right
                6.0, // gears
                0.15); // wheel dia (m)
        m_drive.setDefaultCommand(m_drive.driveManual(
                driverControl::velocity, MAX_SPEED_M_S, MAX_OMEGA_RAD_S));

        m_autons = new Autons(logger, fieldLogger, m_drive);

        new Trigger(driverControl::x).whileTrue(
                m_drive.run(
                        () -> m_drive.setVelocity(new GlobalVelocityR3(1, 0, 0)))
                        .withTimeout(1.0));

        // an example of sequential commands.
        new Trigger(driverControl::y).whileTrue(
                sequence(
                        m_drive.driveWithGlobalVelocity(
                                new GlobalVelocityR3(1.5, 0, 0))
                                .withTimeout(1.0),
                        m_drive.driveWithGlobalVelocity(
                                new GlobalVelocityR3(0, 1.5, 0))
                                .withTimeout(1.0)));
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        Command auton = m_autons.get();
        if (auton == null)
            return;
        auton.schedule();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

}