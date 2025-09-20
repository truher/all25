package org.team100.trajectory_visualizer;

import java.util.List;

import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;

/** This is an example of how we might make a little library of trajectories. */
public class TrajectoryExample {
    public static Trajectory100 oneToTwo() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(1, 1),
                new YawRateConstraint(1, 1));

        TrajectoryPlanner p = new TrajectoryPlanner(c);

        return p.restToRest(
                CanonicalPose.POSE_ONE.pose,
                CanonicalPose.POSE_TWO.pose);
    }

    public static Trajectory100 twoToOne() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(1, 1),
                new YawRateConstraint(1, 1));

        TrajectoryPlanner p = new TrajectoryPlanner(c);

        return p.restToRest(
                CanonicalPose.POSE_TWO.pose,
                CanonicalPose.POSE_ONE.pose);
    }

}
