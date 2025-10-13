package org.team100.ballerina;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.FieldLogger;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.logging.Logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {
    /** Approximately the center of the field */
    private static final Translation2d A = new Translation2d(8, 4);
    /** Some other target. */
    private static final Translation2d B = new Translation2d(4, 2);

    private final DoubleArrayLogger m_log_field_robot;
    private final DoubleArrayLogger m_log_field_target;

    private final XboxController controller;
    private final Turret turret;

    private Pose2d pose;
    private Translation2d target;

    public Robot() {
        Logging logging = Logging.instance();
        logging.setLevel(Level.TRACE);
        LoggerFactory log = logging.rootLogger;
        final LoggerFactory fieldLogger = logging.fieldLogger;
        final FieldLogger.Log fieldLog = new FieldLogger.Log(fieldLogger);
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, "robot");
        m_log_field_target = fieldLogger.doubleArrayLogger(Level.COMP, "target");

        controller = new XboxController(0);
        pose = new Pose2d(6, 4, Rotation2d.kZero);
        target = A;
        turret = new Turret(log, fieldLogger, () -> pose, () -> target);
        // button 3
        new Trigger(controller::getXButton).whileTrue(turret.aim());
        // button 1
        new Trigger(controller::getAButton).onTrue(
                Commands.runOnce(() -> {
                    target = A;
                }));
        // button 2
        new Trigger(controller::getBButton).onTrue(
                Commands.runOnce(() -> {
                    target = B;
                }));
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_log_field_robot.log(() -> new double[] {
                pose.getX(),
                pose.getY(),
                pose.getRotation().getDegrees() });
        m_log_field_target.log(() -> new double[] {
                target.getX(),
                target.getY(),
                0 });
    }

    @Override
    public void teleopInit() {

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

}
