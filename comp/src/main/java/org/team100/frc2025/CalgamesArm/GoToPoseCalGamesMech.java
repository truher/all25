package org.team100.frc2025.CalgamesArm;

import java.util.List;

import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;

import edu.wpi.first.wpilibj2.command.Command;

/** Analogous to DriveWithTrajectory, but for R3 positional control. */
public class GoToPoseCalGamesMech extends Command {

    private final CalgamesMech m_subsystem;
    private final HolonomicPose2d m_holonomicPose2d;
    private final HolonomicPose2d m_currentPose;
    private final TrajectoryPlanner m_trajectoryPlanner;

    private CalgamesReferenceController m_referenceController;

    public GoToPoseCalGamesMech(
            CalgamesMech subsystem,
            HolonomicPose2d holonomicPose2d,
            HolonomicPose2d currentPose,
            TrajectoryPlanner trajectoryPlanner) {
        m_subsystem = subsystem;
        m_holonomicPose2d = holonomicPose2d;
        m_currentPose = currentPose;
        m_trajectoryPlanner = trajectoryPlanner;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        Trajectory100 m_trajectory = m_trajectoryPlanner.restToRest(List.of(
                m_currentPose,
                m_holonomicPose2d));
        m_referenceController = new CalgamesReferenceController(
                m_subsystem,
                new TrajectoryReference(m_trajectory));
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
