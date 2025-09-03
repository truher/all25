package org.team100.frc2025.Swerve.SemiAuto;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.timing.TimingConstraintFactory;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class Navigator extends Command {

    public final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final TrajectoryVisualization m_viz;

    // used by trajectory()
    protected final TrajectoryPlanner m_planner;

    // created in initialize()
    protected ReferenceController m_referenceController;

    public Navigator(
            LoggerFactory parent,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            TrajectoryVisualization viz,
            SwerveKinodynamics kinodynamics) {
        m_drive = drive;
        m_controller = controller;
        m_viz = viz;
        m_planner = new TrajectoryPlanner(new TimingConstraintFactory(kinodynamics).auto());
        addRequirements(m_drive);
    }

    /** Subclasses make trajectories. */
    protected abstract Trajectory100 trajectory(Pose2d currentPose);

    @Override
    public void initialize() {
        System.out.println("I STARTED TO DRIVE");
        Trajectory100 trajectory = trajectory(m_drive.getPose());
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
        System.out.println("I FINISHED");
    }

    public final boolean isDone() {
        return m_referenceController == null || m_referenceController.isFinished();
    }
}
