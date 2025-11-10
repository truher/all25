package org.team100.lib.commands.r3;

import java.util.function.Function;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.commands.r3.helper.VelocityReferenceControllerR3;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.r3.TrajectoryReferenceR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Follow a trajectory created at initialization time, given the pose at that
 * time. Since the trajectory function takes a pose, and not a state, then
 * probably the returned trajectory should start from rest.
 */
public class DriveWithTrajectoryFunction extends MoveAndHold {
    private final LoggerFactory m_log;
    private final VelocitySubsystemR3 m_drive;
    private final ControllerR3 m_controller;
    private final TrajectoryVisualization m_viz;
    private final Function<Pose2d, Trajectory100> m_trajectoryFn;

    /**
     * Non-null when the command is active (between initialize and end), null
     * otherwise.
     */
    private VelocityReferenceControllerR3 m_referenceController;

    public DriveWithTrajectoryFunction(
            LoggerFactory parent,
            VelocitySubsystemR3 drive,
            ControllerR3 controller,
            TrajectoryVisualization viz,
            Function<Pose2d, Trajectory100> trajectoryFn) {
        m_log = parent.type(this);
        m_drive = drive;
        m_controller = controller;
        m_viz = viz;
        m_trajectoryFn = trajectoryFn;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Trajectory100 trajectory = m_trajectoryFn.apply(m_drive.getState().pose());
        m_viz.setViz(trajectory);
        TrajectoryReferenceR3 reference = new TrajectoryReferenceR3(m_log, trajectory);
        m_referenceController = new VelocityReferenceControllerR3(
                m_log, m_drive, m_controller, reference);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
        m_referenceController = null;
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

}
