package org.team100.frc2025.Swerve;

import org.team100.lib.config.ElevatorUtil.ScoringLevel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Positions on the field used as waypoints.
 */
public class FieldConstants {
    /** Staged coral ("lollipop") locations. */
    public enum CoralMark {
        LEFT(new Translation2d(1.219, 5.855)),
        CENTER(new Translation2d(1.219, 4.026)),
        RIGHT(new Translation2d(1.219, 2.197));

        public final Translation2d value;

        private CoralMark(Translation2d t) {
            this.value = t;
        }
    }

    /**
     * Half the distance between the poles, i.e. offset from the center, in meters.
     */
    private static final double REEF_OFFSET = 0.1645; // 0.1524

    private static final Translation2d REEF_CENTER = new Translation2d(4.489, 4.026);

    /**
     * Scoring locations. Single letters are poles for coral, double letters are
     * between poles, for algae.
     */
    public enum ReefPoint {
        A, B, C, D, E, F, G, H, I, J, K, L, AB, CD, EF, GH, IJ, KL;

        /** Direction away from each reef face. */
        Rotation2d angle() {
            return switch (this) {
                case A, B, AB -> Rotation2d.fromDegrees(180);
                case C, D, CD -> Rotation2d.fromDegrees(-120);
                case E, F, EF -> Rotation2d.fromDegrees(-60);
                case G, H, GH -> Rotation2d.fromDegrees(0);
                case I, J, IJ -> Rotation2d.fromDegrees(60);
                case K, L, KL -> Rotation2d.fromDegrees(120);
                default -> throw new IllegalArgumentException();
            };
        }
    }

    public enum CoralStation {
        LEFT, RIGHT;

        public Pose2d pose() {
            return switch (this) {
                case LEFT -> new Pose2d(1.12, 7.14, Rotation2d.fromDegrees(-54)); // 1.2 7.0
                case RIGHT -> new Pose2d(1.12, 0.98, Rotation2d.fromDegrees(54));
                default -> throw new IllegalArgumentException("invalid station");
            };
        }
    }

    public static double getDistanceToReefCenter(Translation2d translation) {
        Translation2d target = REEF_CENTER.minus(translation);
        return target.getNorm();
    }

    public static Rotation2d angleToReefCenter(Translation2d t) {
        Translation2d target = REEF_CENTER.minus(t);
        Rotation2d targetAngle = target.getAngle();
        return targetAngle;
    }

    public static Rotation2d calculateDeltaSpline(Rotation2d originalRotation, Rotation2d deltaRotation, double scale) {
        Translation2d originalTranslation = new Translation2d(1, originalRotation);
        Translation2d deltaTranslation = new Translation2d(scale, deltaRotation);
        Translation2d newTranslation = originalTranslation.plus(deltaTranslation);
        return newTranslation.getAngle();
    }

    public static Pose2d makeGoal(ScoringLevel level, ReefPoint point) {
        double radius = getRadius(point, level);
        Translation2d destination = getScoringDestination(point, radius);
        Rotation2d heading = point.angle().rotateBy(Rotation2d.k180deg);
        return new Pose2d(destination, heading);
    }

    ///////////////////////////

    static Translation2d getScoringDestination(ReefPoint point, double radius) {
        Rotation2d sectorAngle = point.angle();
        Translation2d spoke = new Translation2d(radius, sectorAngle);
        // center of the face of the reef
        Translation2d center = REEF_CENTER.plus(spoke);
        Rotation2d newRotation = sectorAngle.rotateBy(Rotation2d.kCCW_90deg);
        Translation2d d = new Translation2d(REEF_OFFSET, newRotation);
        return switch (point) {
            case A, C, E, G, I, K -> center.minus(d);
            case B, D, F, H, J, L -> center.plus(d);
            case AB, CD, EF, GH, IJ, KL -> center;
            default -> throw new IllegalArgumentException();
        };
    }

    /**
     * Distance to the center of the reef, for each scoring position and level.
     * 
     * These often need to be calibrated for specific reef setups, and to compensate
     * for any lingering inaccuracy in pose estimation.
     */
    private static double getRadius(ReefPoint letter, ScoringLevel level) {
        return switch (level) {
            case L1 -> switch (letter) {
                case A, B, C, D, E, F, G, H, I, J, K, L -> 3;
                case AB, CD, EF, GH, IJ, KL -> 1.2;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case L2 -> switch (letter) {
                case A, B, C, D, E, F, G, H, I, J, K, L -> 1.295;
                case AB, CD, EF, GH, IJ, KL -> 1.2;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case L3 -> switch (letter) {
                case A -> 1.34;
                case AB -> 1.2;
                case B -> 1.34;

                case C -> 1.34;
                case CD -> 1.2;
                case D -> 1.34;

                case E -> 1.34;
                case EF -> 1.2;
                case F -> 1.34;

                case G -> 1.34;
                case GH -> 1.2;
                case H -> 1.34;

                case I -> 1.34;
                case IJ -> 1.2;
                case J -> 1.34;

                case K -> 1.34;
                case KL -> 1.2;
                case L -> 1.34;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case L4 -> switch (letter) {
                case A -> 1.445;
                case AB -> 1.2;
                case B -> 1.445;

                case C -> 1.445;
                case CD -> 1.2;
                case D -> 1.445;

                case E -> 1.445;
                case EF -> 1.2;
                case F -> 1.445;

                case G -> 1.445;
                case GH -> 1.2;
                case H -> 1.445;

                case I -> 1.43;
                case IJ -> 1.2;
                case J -> 1.445;

                case K -> 1.43;
                case KL -> 1.2;
                case L -> 1.43;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case NONE -> 3.0;
            default -> throw new IllegalArgumentException("invalid level");
        };
    }

}
