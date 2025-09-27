package org.team100.frc2025.CalgamesArm;

import java.util.List;

import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;

/** This is an example of how we might make a little library of trajectories. */
public class Trajectories {

    private final TrajectoryPlanner m_planner;

    public Trajectories() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(1, 1),
                new YawRateConstraint(1, 1));
        m_planner = new TrajectoryPlanner(c);
    }


    //trajecotries for moving from wherever to home
    public Trajectory100 whereverToBetween() {
        return m_planner.restToRest(List.of( //pick from back go to home (aka going forward)
                Waypoint.FROM_PICK .hPose,
                Waypoint.GOING_FORWARD.hPose));
    }

    //coral
    public Trajectory100 pickToBetween() {
        return m_planner.restToRest(List.of( //pick from back go to home (aka going forward)
                Waypoint.FROM_PICK.hPose,
                Waypoint.GOING_FORWARD.hPose));
    }

    public Trajectory100 stationToBetween() {
        return m_planner.restToRest(List.of( //pick from back go to home (aka going forward)
                Waypoint.FROM_STATION.hPose,
                Waypoint.GOING_FORWARD.hPose));
    }

    //algae
    public Trajectory100 algaePickToBetween() {
        return m_planner.restToRest(List.of( //pick from back go to home (aka going forward)
                Waypoint.FROM_APICK.hPose,
                Waypoint.GOING_FORWARD.hPose));
    }

    public Trajectory100 A1ToBetween() { //A1 is the top algae
        return m_planner.restToRest(List.of( //pick from back go to home (aka going forward)
                Waypoint.FROM_A1.hPose,
                Waypoint.GOING_FORWARD.hPose));
    }

    public Trajectory100 A2ToBetween() {
        return m_planner.restToRest(List.of( //pick from back go to home (aka going forward)
                Waypoint.FROM_A2.hPose,
                Waypoint.GOING_FORWARD.hPose));
    }

    //all this for home to acquiring
    public Trajectory100 betweenToPick() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_BACKWARD.hPose,
                Waypoint.TO_PICK.hPose));
    }
    public Trajectory100 betweenToStation() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_BACKWARD.hPose,
                Waypoint.TO_STATION.hPose));
    }

    public Trajectory100 betweenToAPICK() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_BACKWARD.hPose,
                Waypoint.TO_APICK.hPose));
    }
    public Trajectory100 betweenToA1() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_BACKWARD.hPose,
                Waypoint.TO_A1.hPose));
    }
    public Trajectory100 betweenToA2() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_BACKWARD.hPose,
                Waypoint.TO_A2.hPose));
    }

    //All this is from home to scoring
    public Trajectory100 betweenToL1() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_FORWARD.hPose,
                Waypoint.TO_L1.hPose));
    }

    public Trajectory100 betweenToL2() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_FORWARD.hPose,
                Waypoint.TO_L2.hPose));
    }

    public Trajectory100 betweenToL3() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_FORWARD.hPose,
                Waypoint.TO_L3.hPose));
    }

    public Trajectory100 betweenToL4() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_FORWARD.hPose,
                Waypoint.TO_L4.hPose));
    }

    public Trajectory100 betweenToBarge() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_FORWARD.hPose,
                Waypoint.TO_BARGE.hPose));
    }

    public Trajectory100 betweenToProcessor() {
        return m_planner.restToRest(List.of(
                Waypoint.GOING_FORWARD.hPose,
                Waypoint.TO_PROCESSOR.hPose));
    }

    //all this is going from scoring to home
    public Trajectory100 L1ToBetween() {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_L1.hPose,
                Waypoint.GOING_BACKWARD.hPose));
    }

    public Trajectory100 L2ToBetween() {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_L2.hPose,
                Waypoint.GOING_BACKWARD.hPose));
    }
    
    public Trajectory100 L3ToBetween() {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_L3.hPose,
                Waypoint.GOING_BACKWARD.hPose));
    }

    public Trajectory100 L4ToBetween() {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_L4.hPose,
                Waypoint.GOING_BACKWARD.hPose));
    }

    public Trajectory100 bargeToBetween() {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_BARGE.hPose,
                Waypoint.GOING_BACKWARD.hPose));
    }

    public Trajectory100 processorToBetween() {
        return m_planner.restToRest(List.of(
                Waypoint.FROM_PROCESSOR.hPose,
                Waypoint.GOING_BACKWARD.hPose));
    }
    
}
