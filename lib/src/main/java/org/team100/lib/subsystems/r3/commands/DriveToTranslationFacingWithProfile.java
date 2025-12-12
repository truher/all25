package org.team100.lib.subsystems.r3.commands;

import java.util.function.Supplier;

import org.team100.lib.commands.MoveAndHold;
import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.profile.r3.ProfileR3;
import org.team100.lib.reference.r3.ProfileReferenceR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;
import org.team100.lib.subsystems.r3.commands.helper.VelocityReferenceControllerR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Drive to a pose supplied at initialization, using a profile.
 */
public class DriveToTranslationFacingWithProfile extends MoveAndHold {
    private final LoggerFactory m_log;
    private final VelocitySubsystemR3 m_drive;
    private final ControllerR3 m_controller;
    private final ProfileR3 m_profile;
    private final Pose2dLogger m_log_goal;
    private final Supplier<Translation2d> m_goal;
    private final Rotation2d m_sideFacing;

    private ProfileReferenceR3 m_reference;
    private VelocityReferenceControllerR3 m_referenceController;

    public DriveToTranslationFacingWithProfile(
            LoggerFactory parent,
            VelocitySubsystemR3 drive,
            ControllerR3 controller,
            ProfileR3 profile,
            Supplier<Translation2d> goal,
            Rotation2d sideFacing) {
        m_log = parent.type(this);
        m_log_goal = m_log.pose2dLogger(Level.TRACE, "goal");
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_goal = goal;
        m_sideFacing = sideFacing;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d goal = getGoal(m_goal.get(),
                m_drive.getState().pose().getTranslation());
        m_log_goal.log(() -> goal);
        m_reference = new ProfileReferenceR3(m_log, m_profile, "embark");
        m_reference.setGoal(new ModelR3(goal));
        m_referenceController = new VelocityReferenceControllerR3(
                m_log, m_drive, m_controller, m_reference);
    }

    @Override
    public void execute() {
        m_referenceController.execute();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.stop();
        m_reference.end();
        m_reference = null;
        m_referenceController = null;
    }

    @Override
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }

    public Pose2d getGoal(Translation2d goalTranslation, Translation2d robotTranslation) {
        return new Pose2d(goalTranslation, goalTranslation.minus(robotTranslation).getAngle().plus(m_sideFacing));
    }

    public double toGo() {
        return m_referenceController.toGo();
    }
}
