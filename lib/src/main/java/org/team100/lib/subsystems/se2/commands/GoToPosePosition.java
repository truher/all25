package org.team100.lib.subsystems.se2.commands;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.se2.TrajectoryReferenceSE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;
import org.team100.lib.subsystems.se2.commands.helper.PositionReferenceControllerSE2;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Using the pose at initialization time, and the specified course, construct a
 * rest-to-rest trajectory to the goal and follow it.
 */
public class GoToPosePosition extends MoveAndHold {
    private final LoggerFactory m_log;
    private final PositionSubsystemSE2 m_subsystem;
    private final WaypointSE2 m_goal;
    private final Rotation2d m_course;
    private final TrajectoryPlanner m_trajectoryPlanner;

    private PositionReferenceControllerSE2 m_referenceController;

    public GoToPosePosition(
            LoggerFactory parent,
            PositionSubsystemSE2 subsystem,
            Rotation2d course,
            WaypointSE2 goal,
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
        WaypointSE2 m_currentPose = new WaypointSE2(
                m_subsystem.getState().pose(),
                DirectionSE2.irrotational(m_course), 1);
        Trajectory100 m_trajectory = m_trajectoryPlanner.restToRest(
                List.of(m_currentPose, m_goal));
        m_referenceController = new PositionReferenceControllerSE2(
                m_log, m_subsystem, new TrajectoryReferenceSE2(m_log, m_trajectory));
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