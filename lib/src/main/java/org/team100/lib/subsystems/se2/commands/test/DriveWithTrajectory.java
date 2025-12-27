package org.team100.lib.subsystems.se2.commands.test;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.se2.TrajectoryReferenceSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.VelocityReferenceControllerSE2;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.visualization.TrajectoryVisualization;

/**
 * Follow a single trajectory.
 * 
 * Because the whole trajectory is fixed, this command can only be used if the
 * robot happens to be in that spot -- and that is often hard to guarantee.
 * 
 * So this is really just for testing.
 */
public class DriveWithTrajectory extends MoveAndHold {
    private final LoggerFactory m_log;
    private final VelocitySubsystemSE2 m_drive;
    private final ControllerSE2 m_controller;
    private final Trajectory100 m_trajectory;
    private final TrajectoryVisualization m_viz;

    private VelocityReferenceControllerSE2 m_referenceController;

    public DriveWithTrajectory(
            LoggerFactory parent,
            VelocitySubsystemSE2 drive,
            ControllerSE2 controller,
            Trajectory100 trajectory,
            TrajectoryVisualization viz) {
        m_log = parent.type(this);
        m_drive = drive;
        m_controller = controller;
        m_trajectory = trajectory;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        TrajectoryReferenceSE2 reference = new TrajectoryReferenceSE2(m_log, m_trajectory);
        m_referenceController = new VelocityReferenceControllerSE2(
                m_log, m_drive, m_controller, reference);
        m_viz.setViz(m_trajectory);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
        return m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
    }
}
