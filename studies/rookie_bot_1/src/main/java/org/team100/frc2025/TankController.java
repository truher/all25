package org.team100.frc2025;

import org.team100.lib.coherence.Takt;
import org.team100.lib.examples.tank.TankDrive;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * see wpi example, simpledifferentialdrivesimulation
 */
public class TankController extends Command {
    private static final boolean DEBUG = true;

    private final Trajectory100 m_trajectory;
    private final TankDrive m_drive;
    private final LTVUnicycleController m_controller;
    private double m_startTimeS;

    public TankController(Trajectory100 trajectory, TankDrive drive) {
        m_trajectory = trajectory;
        m_drive = drive;
        m_controller = new LTVUnicycleController(0.020);
    }

    @Override
    public void initialize() {
        m_startTimeS = Takt.get();
    }

    @Override
    public void execute() {
        TimedPose sample = m_trajectory.sample(progress());
        if (DEBUG)
            System.out.printf("time %5.2f sample %s\n", progress(), sample);
        Pose2d robot = m_drive.getPose();
        ChassisSpeeds speeds = m_controller.calculate(
                robot,
                sample.state().getPose(),
                sample.velocityM_S(),
                sample.state().getCurvature() * sample.velocityM_S());
        m_drive.setVelocity(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    public boolean isDone() {
        return m_trajectory.isDone(progress());
    }

    private double progress() {
        return Takt.get() - m_startTimeS;
    }
}
