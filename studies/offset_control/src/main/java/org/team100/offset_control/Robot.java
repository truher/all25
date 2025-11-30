
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
    private static final double REEF_X = 3.089;
    private static final double REEF_Y = 4.026;
    private final Runnable m_robotViz;
    private final Runnable m_toolpointViz;
    private final DriverXboxControl m_controller;

    public Robot() {
        Banner.printBanner();
        Experiments.instance.show();
        LoggerFactory field = Logging.instance().fieldLogger;
        LoggerFactory log = Logging.instance().rootLogger;

        Translation2d offset = new Translation2d(1, 0);
        TrivialDrivetrain robot = new TrivialDrivetrain(
                new Pose2d(offset.unaryMinus(), Rotation2d.kZero));
        VelocitySubsystemR3 toolpoint = new OffsetDrivetrain(
                robot, offset);

        m_robotViz = new RobotPoseVisualization(
                field, () -> robot.getState().pose(), "robot");
        m_toolpointViz = new RobotPoseVisualization(
                field, () -> toolpoint.getState().pose(), "ctrl");
        m_controller = new DriverXboxControl(0);

        setup1(log, toolpoint);

        setup2(log, toolpoint);
    }

    /** Setup profiles to A and to B */
    private void setup1(LoggerFactory log, VelocitySubsystemR3 toolpoint) {
        // this is intended to be *fast*
        HolonomicProfile profile = HolonomicProfile.currentLimitedExponential(
                5, // v cartesian
                10, // a cartesian
                20, // stall a cartesian
                10, // omega
                10, // alpha
                20); // stall alpha
        // this should try to control cartesian much harder than rotation
        ControllerR3 controller = new FullStateControllerR3(
                log,
                10, // p cartesian
                4, // p theta
                0.055, // p cartesian v
                0.005, // p theta v
                0.03, // x tol
                0.03, // theta tol
                0.1, // xdot tol
                0.1);

        // these aren't actually A and B, they're further apart to make the demo look
        // better.
        Pose2d A = new Pose2d(REEF_X, REEF_Y + 0.5, Rotation2d.kZero);
        Pose2d B = new Pose2d(REEF_X, REEF_Y - 0.5, Rotation2d.kZero);

        DriveToPoseWithProfile toA = new DriveToPoseWithProfile(
                log, toolpoint, controller, profile, () -> A);
        new Trigger(m_controller::a).whileTrue(toA.until(toA::isDone));

        DriveToPoseWithProfile toB = new DriveToPoseWithProfile(
                log, toolpoint, controller, profile, () -> B);
        new Trigger(m_controller::b).whileTrue(toB.until(toB::isDone));
    }

    /** Setup pure PID examples */
    private void setup2(LoggerFactory log, VelocitySubsystemR3 toolpoint) {
        ControllerR3 gentleController = new FullStateControllerR3(
                log,
                3, // p cartesian
                1, // p theta
                0.04, // p cartesian v
                0.01, // p theta v
                0.05, // x tol
                0.05, // theta tol
                1, // xdot tol
                1); // omega tol
        MoveAndHold driveSimple = new DriveToStateSimple(
                log, gentleController, toolpoint, new ModelR3());
        new Trigger(m_controller::x).whileTrue(
                driveSimple.until(driveSimple::isDone));

        MoveAndHold driveSimple2 = new DriveToStateSimple(
                log, gentleController, toolpoint,
                new ModelR3(new Pose2d(4, 4, Rotation2d.kCW_90deg)));
        new Trigger(m_controller::y).whileTrue(
                driveSimple2.until(driveSimple2::isDone));
    }

    @Override
    public void robotPeriodic() {
        Takt.update();
        Cache.refresh();
        CommandScheduler.getInstance().run();
        m_robotViz.run();
        m_toolpointViz.run();
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
