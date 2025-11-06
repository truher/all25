package org.team100.lib.commands.r3;

import java.util.function.BiFunction;
import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.controller.r3.VelocityReferenceControllerR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.r3.TrajectoryReferenceR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.VelocitySubsystemR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Drive from the current state to a field-relative goal.
 * 
 * The goal is supplied at initialization time.
 * 
 * The trajectory is supplied by a function that takes both the current pose and
 * the goal. You could use something like `TrajectoryPlanner.movingToRest()` for
 * this function.
 */
public class DriveToPoseWithTrajectory extends MoveAndHold {
    private final LoggerFactory m_log;
    private final Supplier<Pose2d> m_goal;
    private final VelocitySubsystemR3 m_drive;
    private final BiFunction<ModelR3, Pose2d, Trajectory100> m_trajectories;
    private final ControllerR3 m_controller;
    private final TrajectoryVisualization m_viz;

    private Trajectory100 m_trajectory;

    private VelocityReferenceControllerR3 m_referenceController;

    /**
     * @param trajectories function that takes a start and end pose and returns a
     *                     trajectory between them.
     */
    public DriveToPoseWithTrajectory(
            LoggerFactory parent,
            Supplier<Pose2d> goal,
            VelocitySubsystemR3 drive,
            BiFunction<ModelR3, Pose2d, Trajectory100> trajectories,
            ControllerR3 controller,
            TrajectoryVisualization viz) {
        m_log = parent.type(this);
        m_goal = goal;
        m_drive = drive;
        m_trajectories = trajectories;
        m_controller = controller;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_trajectory = m_trajectories.apply(m_drive.getState(), m_goal.get());
        if (m_trajectory.isEmpty()) {
            m_trajectory = null;
            return;
        }
        TrajectoryReferenceR3 reference = new TrajectoryReferenceR3(m_log, m_trajectory);
        m_referenceController = new VelocityReferenceControllerR3(
                m_log, m_drive, m_controller, reference);
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        if (m_trajectory == null)
            return;
        m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
        return m_trajectory == null
                || m_referenceController == null
                || m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
        m_referenceController = null;
    }
}
