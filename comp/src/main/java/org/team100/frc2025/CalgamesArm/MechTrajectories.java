package org.team100.frc2025.CalgamesArm;

import java.util.ArrayList;
import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.geometry.WaypointSE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.prr.AnalyticalJacobian;
import org.team100.lib.subsystems.prr.ElevatorArmWristKinematics;
import org.team100.lib.subsystems.se2.commands.GoToPosePosition;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.path.PathFactory;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimingConstraint;
import org.team100.lib.trajectory.timing.TorqueConstraint;
import org.team100.lib.trajectory.timing.TrajectoryFactory;
import org.team100.lib.trajectory.timing.YawRateConstraint;

import edu.wpi.first.wpilibj2.command.Command;

/** Make a trajectory from the start to the end and follow it. */
public class MechTrajectories extends Command {

    private final LoggerFactory m_log;
    private final CalgamesMech m_subsystem;
    private final TrajectoryPlanner m_planner;

    public MechTrajectories(
            LoggerFactory parent,
            CalgamesMech mech,
            ElevatorArmWristKinematics k,
            AnalyticalJacobian j) {
        m_log = parent.type(this);
        m_subsystem = mech;
        List<TimingConstraint> c = new ArrayList<>();

        // These are known to work, but suboptimal.
        c.add(new ConstantConstraint(m_log, 10, 5));
        c.add(new YawRateConstraint(m_log, 10, 5));
        // This is new
        c.add(new TorqueConstraint(20));

        // ALERT!
        // The parameters here used to be double these values;
        // These finer grains make smoother paths and schedules but
        // take longer to compute, so if it takes too long, make these
        // numbers bigger!
        TrajectoryFactory trajectoryFactory = new TrajectoryFactory(c);
        PathFactory pathFactory = new PathFactory(0.05, 0.01, 0.01, 0.1);
        m_planner = new TrajectoryPlanner(pathFactory, trajectoryFactory);
    }

    /** A command that goes from the start to the end and then finishes. */
    public Command terminal(String name, WaypointSE2 start, WaypointSE2 end) {

        /** Use the start course and ignore the start pose for now */
        MoveAndHold f = new GoToPosePosition(
                m_log, m_subsystem, start.course().toRotation(), end, m_planner);
        return f
                .until(f::isDone)
                .withName(name);
    }

    /** A command that goes from the start to the end and then waits forever. */
    public MoveAndHold endless(String name, WaypointSE2 start, WaypointSE2 end) {

        /** Use the start course and ignore the start pose for now */
        GoToPosePosition c = new GoToPosePosition(
                m_log, m_subsystem, start.course().toRotation(), end, m_planner);
        c.setName(name);
        return c;

    }

}
