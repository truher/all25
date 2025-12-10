package org.team100.lib.subsystems.r3.commands;

import java.util.Optional;
import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.r3.ProfileReferenceR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;
import org.team100.lib.subsystems.r3.commands.helper.VelocityReferenceControllerR3;
import org.team100.lib.targeting.TargetUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Drive to a robot-relative offset from the supplied target using a profile.
 * 
 * If the supplier starts delivering empties (e.g. the camera loses sight of the
 * goal), retain the old goal (forever).
 */
public class DriveToTranslationWithOffsetUsingProfile extends MoveAndHold {
    /** ignore goals further than this from the previous fix */
    private static final double GOAL_UPDATE_RADIUS = 0.2;
    /** verrrrrry loose. */
    private static final double THETA_TOLERANCE = 0.1;
    private final LoggerFactory m_log;
    private final DoubleArrayLogger m_log_field_goal;
    private final Supplier<Optional<Translation2d>> m_target;
    private final VelocitySubsystemR3 m_drive;
    private final ControllerR3 m_controller;
    private final HolonomicProfile m_profile;
    /** End effector relative to robot */
    private final Transform2d m_endEffector;

    private Pose2d m_goal;
    private ProfileReferenceR3 m_reference;
    private VelocityReferenceControllerR3 m_referenceController;

    public DriveToTranslationWithOffsetUsingProfile(
            LoggerFactory parent,
            LoggerFactory field,
            Supplier<Optional<Translation2d>> target,
            VelocitySubsystemR3 drive,
            ControllerR3 controller,
            HolonomicProfile profile,
            Transform2d endEffector) {
        m_log = parent.type(this);
        m_log_field_goal = field.doubleArrayLogger(Level.COMP, "ball");
        m_target = target;
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_endEffector = endEffector;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        updateGoal();
        if (m_goal == null)
            return;
        m_reference = new ProfileReferenceR3(m_log, m_profile, "DriveToTranslationWithRelativeBearing");
        m_reference.setGoal(new ModelR3(m_goal));
        m_referenceController = new VelocityReferenceControllerR3(
                m_log, m_drive, m_controller, m_reference);
    }

    @Override
    public void execute() {
        updateGoal();
        if (m_goal == null || m_referenceController == null)
            return;
        m_reference.setGoal(new ModelR3(m_goal));
        m_referenceController.execute();
        m_log_field_goal.log(() -> new double[] {
                m_goal.getX(),
                m_goal.getY(),
                m_goal.getRotation().getRadians() });
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

    @Override
    public double toGo() {
        return (m_referenceController == null) ? 0 : m_referenceController.toGo();
    }

    /** For the runway path we can switch to "go to goal" after we're aligned. */
    public boolean thetaAligned() {
        if (m_goal == null)
            return false;
        Pose2d pose = m_drive.getState().pose();
        Rotation2d goalR = m_goal.getRotation();
        Rotation2d poseR = pose.getRotation();
        return Math.abs(goalR.minus(poseR).getRadians()) < THETA_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
        m_goal = null;
    }

    private void updateGoal() {
        Optional<Translation2d> optTarget = m_target.get();
        if (optTarget.isEmpty())
            return;
        Translation2d target = optTarget.get();
        Pose2d robot = m_drive.getState().pose();
        Pose2d candidate = candidate(target, robot, m_endEffector);
        m_goal = filterGoal(candidate);
    }

    static Pose2d candidate(Translation2d target, Pose2d robot, Transform2d offset) {
        Translation2d endEffector = robot.plus(offset).getTranslation();
        Rotation2d endEffectorToTarget = TargetUtil.absoluteBearing(endEffector, target);
        Pose2d targetPose = new Pose2d(target, endEffectorToTarget);
        return targetPose.plus(offset.inverse());
    }

    /**
     * Sometimes we lose sight of the actual closest target, and choose a further
     * away one. In that case, we should ignore the update.
     */
    private Pose2d filterGoal(Pose2d candidate) {
        if (m_goal == null)
            return candidate;
        if (m_goal.getTranslation().getDistance(candidate.getTranslation()) < GOAL_UPDATE_RADIUS)
            return candidate;
        return m_goal;
    }
}
