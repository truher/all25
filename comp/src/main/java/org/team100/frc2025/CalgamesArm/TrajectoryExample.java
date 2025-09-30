package org.team100.frc2025.CalgamesArm;

import java.util.List;

import org.team100.lib.geometry.HolonomicPose2d;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.YawRateConstraint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** This is an example of how we might make a little library of trajectories. */
public class TrajectoryExample {

    private final TrajectoryPlanner m_planner;

    public TrajectoryExample() {
        List<TimingConstraint> c = List.of(
                new ConstantConstraint(1, 1),
                new YawRateConstraint(1, 1));
        m_planner = new TrajectoryPlanner(c);
    }

    public Trajectory100 homeToL4() {
        return m_planner.restToRest(List.of(
                new HolonomicPose2d(
                        new Translation2d(1, 0),
                        Rotation2d.kZero,
                        Rotation2d.kZero),
                Waypoint.TO_L4.hPose));
    }

}
