package org.team100.lib.commands.drivetrain.tank;

import org.team100.lib.coherence.Takt;
import org.team100.lib.framework.TimedRobot100;
import org.team100.lib.motion.tank.TankDrive;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Follows a trajectory with the WPI LTVUnicycleController, which combines
 * (reference velocity passthrough) feedforward and feedback. The feedback has
 * two independent components:
 * 
 * * translation: robot-relative X (fore/aft) error
 * * rotation: a mix of robot-relative Y (sideways) and rotation errors
 * 
 * The rotation error has a greater effect at greater speeds.
 */
public class FixedTrajectory extends Command {
    private final Trajectory100 m_trajectory;
    private final TankDrive m_drive;
    private final LTVUnicycleController m_controller;
    private double m_startTimeS;

    public FixedTrajectory(Trajectory100 trajectory, TankDrive drive) {
        m_trajectory = trajectory;
        m_drive = drive;
        m_controller = new LTVUnicycleController(0.020);
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_startTimeS = Takt.get();
    }

    @Override
    public void execute() {
        // current for position error
        double t = progress();
        TimedPose current = m_trajectory.sample(t);
        // next for feedforward (and selecting K)
        TimedPose next = m_trajectory.sample(t + TimedRobot100.LOOP_PERIOD_S);
        Pose2d currentPose = m_drive.getPose();
        Pose2d poseReference = current.state().getPose();
        double velocityReference = next.velocityM_S();
        double omegaReference = next.velocityM_S() * next.state().getCurvature();
        ChassisSpeeds speeds = m_controller.calculate(
                currentPose, poseReference, velocityReference, omegaReference);
        m_drive.setVelocity(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    /** Done when the timer expires. Ignores actual position */
    public boolean isDone() {
        return m_trajectory.isDone(progress());
    }

    /** Time since start */
    private double progress() {
        return Takt.get() - m_startTimeS;
    }
}
