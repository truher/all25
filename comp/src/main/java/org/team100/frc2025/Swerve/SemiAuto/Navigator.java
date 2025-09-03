package org.team100.frc2025.Swerve.SemiAuto;

import java.util.function.Function;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Navigator extends Command {
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final TrajectoryVisualization m_viz;
    private final Function<Pose2d, Trajectory100> m_trajectoryFn;

    /**
     * Non-null when the command is active (between initialize and end), null
     * otherwise.
     */
    private ReferenceController m_referenceController;

    public Navigator(
            SwerveDriveSubsystem drive,
            SwerveController controller,
            TrajectoryVisualization viz,
            Function<Pose2d, Trajectory100> trajectoryFn) {
        m_drive = drive;
        m_controller = controller;
        m_viz = viz;
        m_trajectoryFn = trajectoryFn;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Trajectory100 trajectory = m_trajectoryFn.apply(m_drive.getPose());
        m_viz.setViz(trajectory);
        m_referenceController = new ReferenceController(
                m_drive,
                m_controller,
                new TrajectoryReference(trajectory), true);
    }

    @Override
    public final void execute() {
        m_referenceController.execute();
    }

    @Override
    public final void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
        m_referenceController = null;
    }

    /**
     * Done if we've started and we're finished.
     * Note calling isDone after end will yield false.
     */
    public final boolean isDone() {
        return m_referenceController != null && m_referenceController.isFinished();
    }
}
