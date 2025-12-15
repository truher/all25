package org.team100.lib.subsystems.r3.commands;

import java.util.List;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.geometry.DirectionSE2;
import org.team100.lib.geometry.Pose2dWithDirection;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.r3.TrajectoryReferenceR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;
import org.team100.lib.subsystems.r3.commands.helper.VelocityReferenceControllerR3;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.visualization.TrajectoryVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Drive to a specified pose and exit velocity, using a trajectory constructed
 * at initialization time.
 */
public class DriveToPoseWithTrajectoryAndExitVelocity extends MoveAndHold {
    private final LoggerFactory m_log;
    private final Pose2d m_goal;
    private final VelocitySE2 m_endVelocity;
    private final VelocitySubsystemR3 m_drive;
    private final ControllerR3 m_controller;
    private final TrajectoryVisualization m_viz;
    private final TrajectoryPlanner m_planner;

    private VelocityReferenceControllerR3 m_referenceController;

    public DriveToPoseWithTrajectoryAndExitVelocity(
            LoggerFactory parent,
            Pose2d goal,
            VelocitySE2 endVelocity,
            VelocitySubsystemR3 drive,
            ControllerR3 controller,
            TrajectoryPlanner planner,
            TrajectoryVisualization viz) {
        m_log = parent.type(this);
        m_goal = goal;
        m_endVelocity = endVelocity;
        m_drive = drive;
        m_controller = controller;
        m_planner = planner;
        m_viz = viz;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d pose = m_drive.getState().pose();
        Translation2d toGoal = m_goal.getTranslation().minus(pose.getTranslation());
        VelocitySE2 startVelocity = m_drive.getState().velocity();
        Pose2dWithDirection startWaypoint = new Pose2dWithDirection(
                pose,
                DirectionSE2.fromRotation(startVelocity.angle().orElse(toGoal.getAngle())));
        Pose2dWithDirection endWaypoint = new Pose2dWithDirection(
                m_goal,
                DirectionSE2.fromRotation(m_endVelocity.angle().orElse(toGoal.getAngle())));
        Trajectory100 trajectory = m_planner.generateTrajectory(
                List.of(startWaypoint, endWaypoint),
                startVelocity.norm(),
                m_endVelocity.norm());

        if (trajectory.length() == 0) {
            m_referenceController = null;
            return;
        }

        m_viz.setViz(trajectory);

        TrajectoryReferenceR3 reference = new TrajectoryReferenceR3(m_log, trajectory);
        m_referenceController = new VelocityReferenceControllerR3(
                m_log, m_drive, m_controller, reference);
    }

    @Override
    public void execute() {
        if (m_referenceController != null)
            m_referenceController.execute();

    }

    @Override
    public boolean isDone() {
        if (m_referenceController == null)
            return true;
        return m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_viz.clear();
    }
}
