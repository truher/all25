package org.team100.lib.commands.r3;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.PositionReferenceControllerR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.r3.TrajectoryReferenceR3;
import org.team100.lib.subsystems.PositionSubsystemR3;
import org.team100.lib.trajectory.Trajectory100;

/** Analogous to DriveWithTrajectory, but for R3 positional control. */
public class FollowTrajectoryPosition extends MoveAndHold {
    private final LoggerFactory m_log;
    private final PositionSubsystemR3 m_subsystem;
    private final Trajectory100 m_trajectory;

    private PositionReferenceControllerR3 m_referenceController;

    public FollowTrajectoryPosition(
            LoggerFactory parent,
            PositionSubsystemR3 subsystem,
            Trajectory100 trajectory) {
        m_log = parent.type(this);
        m_subsystem = subsystem;
        m_trajectory = trajectory;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_referenceController = new PositionReferenceControllerR3(
                m_log, m_subsystem, new TrajectoryReferenceR3(m_log, m_trajectory));
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
        m_subsystem.stop();
    }

}
