package org.team100.lib.commands.r3;

import org.team100.lib.controller.r3.ReferenceControllerR3;
import org.team100.lib.reference.TrajectoryReference;
import org.team100.lib.trajectory.Trajectory100;

import edu.wpi.first.wpilibj2.command.Command;

/** Analogous to DriveWithTrajectory, but for R3 positional control. */
public class FollowTrajectory extends Command {

    private final SubsystemR3 m_subsystem;
    private final Trajectory100 m_trajectory;

    private ReferenceControllerR3 m_referenceController;

    public FollowTrajectory(
            SubsystemR3 subsystem,
            Trajectory100 trajectory) {
        m_subsystem = subsystem;
        m_trajectory = trajectory;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        m_referenceController = new ReferenceControllerR3(
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
