package org.team100.trajectory_visualizer;

import java.util.List;

import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;

/** This is an example of how we might make a little library of trajectories. */
public class TrajectoryExample {

    private final TrajectoryPlanner m_planner;

    public TrajectoryExample() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(1, 1),
                new YawRateConstraint(1, 1));
        m_planner = new TrajectoryPlanner(c);
    }

    public Trajectory100 pickToL4() {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_PICK.hPose,
                Waypoint.GOING_FORWARD.hPose,
                Waypoint.TO_L4.hPose));
    }

    public Trajectory100 pickToL3() {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_PICK.hPose,
                Waypoint.GOING_FORWARD.hPose,
                Waypoint.TO_L3.hPose));
    }

    public Trajectory100 pickToL2() {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_PICK.hPose,
                Waypoint.GOING_FORWARD.hPose,
                Waypoint.TO_L2.hPose));
    }

    public Trajectory100 pickToL1() {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_PICK.hPose,
                Waypoint.GOING_FORWARD.hPose,
                Waypoint.TO_L1.hPose));
    }

}
