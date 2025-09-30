package org.team100.frc2025.CalgamesArm;

import java.util.List;

import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;

public class Trajectories {

    public static Trajectory100 A1ToBetween(TrajectoryPlanner m_planner) {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_A1.hPose,
                Waypoint.GOING_FORWARD.hPose));
    }

    public static Trajectory100 A2ToBetween(TrajectoryPlanner m_planner) {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_A2.hPose,
                Waypoint.GOING_FORWARD.hPose));
    }

    public static Trajectory100 bargeToBetween(TrajectoryPlanner m_planner) {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_BARGE.hPose,
                Waypoint.GOING_BACKWARD.hPose));
    }



}
