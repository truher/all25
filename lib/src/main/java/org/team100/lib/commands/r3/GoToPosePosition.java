package org.team100.lib.commands.r3;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.PositionReferenceControllerR3;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.r3.TrajectoryReferenceR3;
import org.team100.lib.subsystems.r3.PositionSubsystemR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Using the pose at initialization time, and the specified course, construct a
 * rest-to-rest trajectory to the goal and follow it.
 */
public class GoToPosePosition extends MoveAndHold {
    private final LoggerFactory m_log;
    private final PositionSubsystemR3 m_subsystem;
    private final HolonomicPose2d m_goal;
    private final Rotation2d m_course;
    private final TrajectoryPlanner m_trajectoryPlanner;

    private PositionReferenceControllerR3 m_referenceController;

    public GoToPosePosition(
            LoggerFactory parent,
            PositionSubsystemR3 subsystem,
            Rotation2d course,
            HolonomicPose2d goal,
            TrajectoryPlanner trajectoryPlanner) {
        m_log = parent.type(this);
        m_subsystem = subsystem;
        m_goal = goal;
        m_course = course;
        m_trajectoryPlanner = trajectoryPlanner;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        HolonomicPose2d m_currentPose = HolonomicPose2d.make(
                m_subsystem.getState().pose(), m_course);
        Trajectory100 m_trajectory = m_trajectoryPlanner.restToRest(
                List.of(m_currentPose, m_goal));
        m_referenceController = new PositionReferenceControllerR3(
                m_log, m_subsystem, new TrajectoryReferenceR3(m_log, m_trajectory));
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public boolean isDone() {
        if (m_referenceController == null)
            return false;
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