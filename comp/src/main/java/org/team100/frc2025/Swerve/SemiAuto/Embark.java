package org.team100.frc2025.Swerve.SemiAuto;

import java.util.function.DoubleConsumer;
import java.util.function.Supplier;

import org.team100.lib.commands.drivetrain.FieldConstants;
import org.team100.lib.commands.drivetrain.FieldConstants.FieldSector;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefDestination;
import org.team100.lib.commands.drivetrain.FieldConstants.ReefPoint;
import org.team100.lib.config.ElevatorUtil.ScoringPosition;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Drive *perpetually* to the supplied pose using a profile.
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
    private final FieldSector m_targetSector;
    private final ReefDestination m_destination;
    private final Supplier<ScoringPosition> m_scoringPositionSupplier;
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
            Supplier<ScoringPosition> scoringPositionSupplier,
            ReefPoint point) {
        LoggerFactory child = logger.type(this);
        m_log_goal = child.pose2dLogger(Level.TRACE, "goal");
        m_drive = drive;
        m_heedRadiusM = heedRadiusM;
        m_controller = controller;
        m_profile = profile;
        m_scoringPositionSupplier = scoringPositionSupplier;
        m_destination = ReefDestination.fromReefPoint(point);
        m_point = point;
        m_targetSector = FieldSector.fromReefPoint(point);
        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_heedRadiusM.accept(HEED_RADIUS_M);
        Pose2d goal = makeGoal(
                m_drive.getPose(), m_targetSector, m_destination, m_scoringPositionSupplier.get(), m_point);
        m_log_goal.log(() -> goal);
        m_reference = new ProfileReference(m_profile);
        m_reference.setGoal(new SwerveModel(goal));
        m_referenceController = new ReferenceController(m_drive, m_controller, m_reference, false);
    }

    private static Pose2d makeGoal(
            Pose2d currentPose,
            FieldSector targetSector,
            ReefDestination reefDestination,
            ScoringPosition scoringPosition,
            ReefPoint point) {
        double radius = getRadius(reefDestination, point, scoringPosition);
        Translation2d destination = FieldConstants.getScoringDestination(targetSector, reefDestination, radius);
        Rotation2d heading = FieldConstants.getSectorAngle(targetSector).rotateBy(Rotation2d.fromDegrees(180));
        return new Pose2d(destination, heading);
    }

    private static double getRadius(ReefDestination pole, ReefPoint letter, ScoringPosition level) {
        if (pole == ReefDestination.CENTER) {
            // for algae
            return 1.2;
        }
        return switch (level) {
            case L1 -> 3;
            case L2 -> 1.295;
            case L3 -> switch (letter) {
                case A -> 1.34;
                case B -> 1.34;
                case C -> 1.34;
                case D -> 1.34;
                case E -> 1.34;
                case F -> 1.34;
                case G -> 1.34;
                case H -> 1.34;
                case I -> 1.34;
                case J -> 1.34;
                case K -> 1.34;
                case L -> 1.34;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case L4 -> switch (letter) {
                case A -> 1.445;
                case B -> 1.445;
                case C -> 1.445;
                case D -> 1.445;
                case E -> 1.445;
                case F -> 1.445;
                case G -> 1.445;
                case H -> 1.445;
                case I -> 1.43;
                case J -> 1.445;
                case K -> 1.43;
                case L -> 1.43;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case NONE -> 3.0;
            default -> throw new IllegalArgumentException("invalid level");
        };
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
