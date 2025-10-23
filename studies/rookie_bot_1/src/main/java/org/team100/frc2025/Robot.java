package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.List;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.examples.tank.DriveTank;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.examples.tank.TankDriveFactory;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
                60, // supply current
                new CanId(6), // left
                new CanId(5), // right
                6.0, // gear ratio
                0.15); // wheel dia (m)
        m_drive.setDefaultCommand(
                new DriveTank(() -> -1.0 * driverControl.rightY(),
                        () -> -1.0 * driverControl.rightX(),
                        m_drive));
        TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(1, 1)));
        // remember that heading and course need to be the same
        List<HolonomicPose2d> waypoints = List.of(
                new HolonomicPose2d(new Translation2d(), Rotation2d.kZero, Rotation2d.kZero),
                new HolonomicPose2d(new Translation2d(1, 1), Rotation2d.kCCW_90deg, Rotation2d.kCCW_90deg),
                new HolonomicPose2d(new Translation2d(2, 2), Rotation2d.kZero, Rotation2d.kZero));
        Trajectory100 trajectory = planner.restToRest(waypoints);
        TankController c1 = new TankController(trajectory, m_drive);

        new Trigger(driverControl::a).whileTrue(c1.until(c1::isDone));

        m_auton = sequence(
                m_drive.run(() -> m_drive.setVelocity(1, 0)).withTimeout(1),
                m_drive.run(() -> m_drive.setVelocity(1, 1)).withTimeout(1),
                m_drive.run(() -> m_drive.setVelocity(1, 0)).withTimeout(1));
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