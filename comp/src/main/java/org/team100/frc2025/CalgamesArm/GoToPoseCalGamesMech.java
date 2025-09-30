package org.team100.frc2025.CalgamesArm;

import java.util.List;

import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** Starting pose is current pose, with the fixed starting course. */
public class GoToPoseCalGamesMech extends Command {

    private final CalgamesMech m_subsystem;
    private final HolonomicPose2d m_goal;
    private final Rotation2d m_course;
    private final TrajectoryPlanner m_trajectoryPlanner;

    private CalgamesReferenceController m_referenceController;

    public GoToPoseCalGamesMech(
            CalgamesMech subsystem,
            Rotation2d course,
            HolonomicPose2d goal,
            TrajectoryPlanner trajectoryPlanner) {
        m_subsystem = subsystem;
        m_goal = goal;
        m_course = course;
        m_trajectoryPlanner = trajectoryPlanner;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        HolonomicPose2d m_currentPose = HolonomicPose2d.fromPose(
                m_subsystem.getState().pose(), m_course);
        Trajectory100 m_trajectory = m_trajectoryPlanner.restToRest(
                List.of(m_currentPose, m_goal));
        m_referenceController = new CalgamesReferenceController(
                m_subsystem, new TrajectoryReference(m_trajectory));
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    public boolean isDone() {
        return m_referenceController.isDone();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop();
    }

}
