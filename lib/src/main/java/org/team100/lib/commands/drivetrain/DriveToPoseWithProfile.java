package org.team100.lib.commands.drivetrain;

import java.util.function.Supplier;

import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.ProfileReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive to a pose supplied at initialization, using a profile.
 */
public class DriveToPoseWithProfile extends Command {
    private final SwerveDriveSubsystem m_drive;
    private final SwerveController m_controller;
    private final HolonomicProfile m_profile;
    private final Pose2dLogger m_log_goal;
    private final Supplier<Pose2d> m_goal;

    private ProfileReference m_reference;
    private ReferenceController m_referenceController;

    public DriveToPoseWithProfile(
            LoggerFactory logger,
            SwerveDriveSubsystem drive,
            SwerveController controller,
            HolonomicProfile profile,
            Supplier<Pose2d> goal) {
        LoggerFactory child = logger.type(this);
        m_log_goal = child.pose2dLogger(Level.TRACE, "goal");
        m_drive = drive;
        m_controller = controller;
        m_profile = profile;
        m_goal = goal;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        Pose2d goal = m_goal.get();
        m_log_goal.log(() -> goal);
        m_reference = new ProfileReference(m_profile, "embark");
        m_reference.setGoal(new SwerveModel(goal));
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
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

    /**
     * Done if we've started and we're finished.
     * Note calling isDone after end will yield false.
     */
    public boolean isDone() {
        return m_referenceController != null && m_referenceController.isDone();
    }
}
