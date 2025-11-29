
package org.team100.offset_control;

import org.team100.lib.coherence.Cache;
import org.team100.lib.coherence.Takt;
import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.controller.r3.FullStateControllerR3;
import org.team100.lib.experiments.Experiments;
import org.team100.lib.hid.DriverXboxControl;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.Logging;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;
import org.team100.lib.subsystems.r3.commands.DriveToPoseWithProfile;
import org.team100.lib.subsystems.r3.commands.test.DriveToStateSimple;
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
    private final Runnable m_ctrlViz;
    private final DriverXboxControl m_controller;

    public Robot() {
        Banner.printBanner();
        Experiments.instance.show();
        Logging log = Logging.instance();
        LoggerFactory fieldLogger = log.fieldLogger;
        LoggerFactory rootLogger = log.rootLogger;
        // real robot starts at (-4,0)
        TrivialDrivetrain delegate = new TrivialDrivetrain(
                new Pose2d(-4, 0, Rotation2d.kZero));
        // offset is x+4 so tool point should be at zero
        m_subsystem = new OffsetDrivetrain(
                delegate, new Translation2d(4, 0));
        m_robotViz = new RobotPoseVisualization(
                fieldLogger,
                () -> delegate.getState().pose(),
                "robot");
        m_ctrlViz = new RobotPoseVisualization(
                fieldLogger,
                () -> m_subsystem.getState().pose(),
                "ctrl");
        m_controller = new DriverXboxControl(0);

        HolonomicProfile profile = HolonomicProfile.currentLimitedExponential(
                1, 2, 4, 5, 10, 5);
        ControllerR3 controller = new FullStateControllerR3(
                rootLogger,
                7.2, // p cartesian
                3.5, // p theta
                0.055, // p cartesian v
                0.01, // p theta v
                0.035, // x tol
                0.1, // theta tol
                1, // xdot tol
                1);

        DriveToPoseWithProfile driveCmd = new DriveToPoseWithProfile(
                rootLogger, m_subsystem, controller,
                profile,
                () -> new Pose2d(4, 4, Rotation2d.kCW_90deg));
        new Trigger(m_controller::a).whileTrue(
                driveCmd.until(driveCmd::isDone));

        ControllerR3 gentleController = new FullStateControllerR3(
                rootLogger,
                3, // p cartesian
                1, // p theta
                0.04, // p cartesian v
                0.01, // p theta v
                0.05, // x tol
                0.05, // theta tol
                1, // xdot tol
                1); // omega tol
        MoveAndHold driveSimple = new DriveToStateSimple(
                rootLogger, gentleController, m_subsystem, new ModelR3());
        new Trigger(m_controller::b).whileTrue(
                driveSimple.until(driveSimple::isDone));

        MoveAndHold driveSimple2 = new DriveToStateSimple(
                rootLogger, gentleController, m_subsystem,
                new ModelR3(new Pose2d(4, 4, Rotation2d.kCW_90deg)));
        new Trigger(m_controller::x).whileTrue(
                driveSimple2.until(driveSimple2::isDone));
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_robotViz.run();
        m_ctrlViz.run();
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
