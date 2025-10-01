package org.team100.frc2025.CalgamesArm;

import java.util.List;

import org.team100.lib.commands.Done;
import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;

import edu.wpi.first.wpilibj2.command.Command;

/** Make a trajectory from the start to the end and follow it. */
public class MechTrajectories extends Command {
    private final CalgamesMech m_subsystem;
    private final TrajectoryPlanner m_planner;

    public MechTrajectories(CalgamesMech mech) {
        m_subsystem = mech;
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(5, 10),
                new YawRateConstraint(5, 5));
        m_planner = new TrajectoryPlanner(c);
    }

    /** A command that goes from the start to the end and then finishes. */
    public Command terminal(HolonomicPose2d start, HolonomicPose2d end) {

        // FollowTrajectory f = new FollowTrajectory(
        // m_subsystem, m_planner.restToRest(List.of(start, end)));

        /** Use the start course and ignore the start pose for now */
        Done f = new GoToPoseCalGamesMech(m_subsystem, start.course(), end, m_planner);
        return f.until(f::isDone);
    }

    /** A command that goes from the start to the end and then waits forever. */
    public Done endless(HolonomicPose2d start, HolonomicPose2d end) {

        /** Use the start course and ignore the start pose for now */
        return new GoToPoseCalGamesMech(m_subsystem, start.course(), end, m_planner);

        // return new FollowTrajectory(m_subsystem, m_planner.restToRest(List.of(start,
        // end)));
    }

}
