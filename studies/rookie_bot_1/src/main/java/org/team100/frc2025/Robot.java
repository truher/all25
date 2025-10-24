package org.team100.frc2025;

import static edu.wpi.first.wpilibj2.command.Commands.sequence;

import java.util.List;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.commands.drivetrain.tank.FixedTrajectory;
import org.team100.lib.config.AnnotatedCommand;
import org.team100.lib.examples.tank.DriveTank;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.examples.tank.TankDriveFactory;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.indicator.Alerts;
import org.team100.lib.indicator.SolidIndicator;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.util.Banner;
import org.team100.lib.util.CanId;
import org.team100.lib.util.RoboRioChannel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot100 {
    private final TankDrive m_drive;
    private final Command m_auton;
    private final Autons m_autons;
    private final SolidIndicator m_indicator;
    private final Alerts m_alerts;
    private final Alert m_noStartingPosition;
    private final Alert m_mismatchedAlliance;

    public Robot() {
        m_indicator = new SolidIndicator(new RoboRioChannel(0), 40);
        m_alerts = new Alerts();
        m_noStartingPosition = m_alerts.add("No starting position!", AlertType.kWarning);
        m_mismatchedAlliance = m_alerts.add("Wrong Alliance!", AlertType.kWarning);
        m_indicator.state(() -> m_alerts.any() ? Color.kRed : Color.kGreen);
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
                        m_drive)
                        .withName("drive default"));
        TrajectoryPlanner planner = new TrajectoryPlanner(
                List.of(new ConstantConstraint(1, 1)));
        List<HolonomicPose2d> waypoints = List.of(
                HolonomicPose2d.tank(0, 0, 0),
                HolonomicPose2d.tank(1, 1, Math.PI / 2),
                HolonomicPose2d.tank(2, 2, 0));
        Trajectory100 trajectory = planner.restToRest(waypoints);
        FixedTrajectory c1 = new FixedTrajectory(trajectory, m_drive);

        new Trigger(driverControl::a).whileTrue(c1.until(c1::isDone));

        m_autons = new Autons(m_drive);

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
        // choose a command at auton init time
        Command auton = m_autons.get().command();
        if (auton == null)
            return;
        auton.schedule();
    }

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void disabledPeriodic() {
        checkStart();
        checkAlliance();
    }

    private void checkStart() {
        AnnotatedCommand cmd = m_autons.get();
        Pose2d start = cmd.start();
        if (start == null) {
            m_noStartingPosition.set(true);
        } else {
            m_noStartingPosition.set(false);
            m_drive.setPose(start);
        }
    }

    private void checkAlliance() {
        AnnotatedCommand cmd = m_autons.get();
        Alliance alliance = cmd.alliance();
        if (alliance == null) {
            // works for either
            return;
        }
        Alliance dsAlliance = DriverStation.getAlliance().orElse(null);
        if (dsAlliance == null) {
            // not set yet
            return;
        }
        if (alliance != dsAlliance) {
            m_mismatchedAlliance.set(true);
        } else {
            m_mismatchedAlliance.set(false);
        }
    }

}