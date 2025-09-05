package org.team100.frc2025.Swerve.SemiAuto;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.frc2025.Swerve.FieldConstants;
import org.team100.frc2025.Swerve.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringLevel;
import org.team100.lib.controller.drivetrain.ReferenceController;
import org.team100.lib.controller.drivetrain.SwerveController;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Pose2dLogger;
import org.team100.lib.motion.drivetrain.SwerveDriveSubsystem;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.profile.HolonomicProfile;
import org.team100.lib.reference.ProfileReference;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive *perpetually* to the supplied pose in a straight line using a holonomic
 * profile.
 * 
 * If the supplier starts delivering empties, retain the old goal.
 */
public class Embark extends Command {
    /** While driving to scoring tag, pay attention only to very close tags. */
    private static final double HEED_RADIUS_M = 3;

    private final SwerveDriveSubsystem m_drive;
    private final DoubleConsumer m_heedRadiusM;
    private final SwerveController m_controller;
    private final HolonomicProfile m_profile;
    private final Supplier<ScoringLevel> m_level;
    private final Pose2dLogger m_log_goal;
    private final ReefPoint m_point;

    private ProfileReference m_reference;
    private ReferenceController m_referenceController;

    public Embark(
            LoggerFactory logger,
            SwerveDriveSubsystem drive,
            DoubleConsumer heedRadiusM,
            SwerveController controller,
            HolonomicProfile profile,
            Supplier<ScoringLevel> level,
            ReefPoint point) {
        LoggerFactory child = logger.type(this);
        m_log_goal = child.pose2dLogger(Level.TRACE, "goal");
        m_drive = drive;
        m_heedRadiusM = heedRadiusM;
        m_controller = controller;
        m_profile = profile;
        m_level = level;
        m_point = point;
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        Pose2d goal = FieldConstants.makeGoal(m_level.get(), m_point);
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
        return m_referenceController != null && m_referenceController.isFinished();
    }
}
