
package org.team100.offset_control;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.controller.r3.ControllerFactoryR3;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;
import org.team100.lib.subsystems.r3.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.test.OffsetDrivetrain;
import org.team100.lib.subsystems.test.TrivialDrivetrain;
import org.team100.lib.util.Banner;
import org.team100.lib.visualization.RobotPoseVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Robot extends TimedRobot {
    private final VelocitySubsystemR3 m_subsystem;
    private final Runnable m_robotViz;
    private final DriverXboxControl m_controller;

    public Robot() {
        Banner.printBanner();
        Experiments.instance.show();
        Logging log = Logging.instance();
        LoggerFactory fieldLogger = log.fieldLogger;
        LoggerFactory rootLogger = log.rootLogger;
        m_subsystem = new OffsetDrivetrain(
                new TrivialDrivetrain(),
                new Translation2d(4, 0));
        m_robotViz = new RobotPoseVisualization(
                fieldLogger, () -> m_subsystem.getState().pose());
        m_controller = new DriverXboxControl(0);

        HolonomicProfile profile = HolonomicProfile.currentLimitedExponential(
                1, 2, 4, 5, 10, 5);
        ControllerR3 controller = ControllerFactoryR3
                .auto2025LooseTolerance(rootLogger);
        DriveToPoseWithProfile driveCmd = new DriveToPoseWithProfile(
                rootLogger, m_subsystem, controller,
                profile,
                () -> new Pose2d(4, 4, Rotation2d.kZero));

        new Trigger(m_controller::a).whileTrue(
                driveCmd.until(driveCmd::isDone));
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_robotViz.run();
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
