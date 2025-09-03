package org.team100.lib.commands.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldConstants {
    /**
     * Half the distance between the poles, i.e. offset from the center, in meters.
     */
    private static final double REEF_OFFSET = 0.1645; // 0.1524

    /** Which face of the reef. */
    public enum FieldSector {
        AB, CD, EF, GH, IJ, KL;

        public static FieldSector fromReefPoint(ReefPoint p) {
            return switch (p) {
                case A, B -> AB;
                case C, D -> CD;
                case E, F -> EF;
                case G, H -> GH;
                case I, J -> IJ;
                case K, L -> KL;
                default -> throw new IllegalArgumentException();
            };
        }
    }

    /** Which pole on the circle. */
    public enum ReefPoint {
        A, B, C, D, E, F, G, H, I, J, K, L
    }

    /** Which pole when you're in front of the tag, or center for algae. */
    public enum ReefDestination {
        LEFT, RIGHT, CENTER;

        public static ReefDestination fromReefPoint(ReefPoint p) {
            return switch (p) {
                case A, C, E, G, I, K -> LEFT;
                case B, D, F, H, J, L -> RIGHT;
                default -> throw new IllegalArgumentException();
            };
        }
    }

    public enum BargeDestination {
        LEFT, RIGHT, CENTER
    }

    public enum ReefAproach {
        CCW, CW
    }

    public enum CoralStation {
        Left, Right
    }

    /** Direction away from each reef face. */
    public static Rotation2d getSectorAngle(FieldSector sector) {
        return switch (sector) {
            case AB -> Rotation2d.fromDegrees(180);
            case CD -> Rotation2d.fromDegrees(-120);
            case EF -> Rotation2d.fromDegrees(-60);
            case GH -> Rotation2d.fromDegrees(0);
            case IJ -> Rotation2d.fromDegrees(60);
            case KL -> Rotation2d.fromDegrees(120);
            default -> throw new IllegalArgumentException();
        };
    }

    public static double getDistanceToReefCenter(Translation2d translation) {
        Translation2d target = FieldConstants.getReefCenter().minus(translation);
        return target.getNorm();
    }

    public static Rotation2d angleToReefCenter(Translation2d t) {
        Translation2d target = FieldConstants.getReefCenter().minus(t);
        Rotation2d targetAngle = target.getAngle();
        return targetAngle;
    }

    public static Translation2d getReefCenter() { // blue
        return new Translation2d(4.489, 4.026);
    }

    public static Rotation2d calculateDeltaSpline(Rotation2d originalRotation, Rotation2d deltaRotation, double scale) {
        Translation2d originalTranslation = new Translation2d(1, originalRotation);
        Translation2d deltaTranslation = new Translation2d(scale, deltaRotation);
        Translation2d newTranslation = originalTranslation.plus(deltaTranslation);
        return newTranslation.getAngle();
    }

    public static Translation2d getScoringDestination(
            FieldSector destinationSector,
            ReefDestination destinationPoint,
            double radius) {
        Rotation2d sectorAngle = getSectorAngle(destinationSector);
        Translation2d spoke = new Translation2d(radius, sectorAngle);
        Translation2d center = getReefCenter().plus(spoke);
        Rotation2d newRotation = sectorAngle.rotateBy(Rotation2d.kCCW_90deg);
        Translation2d d = new Translation2d(REEF_OFFSET, newRotation);
        return switch (destinationPoint) {
            case RIGHT -> center.plus(d);
            case LEFT -> center.minus(d);
            case CENTER -> center;
            default -> throw new IllegalArgumentException();
        };
    }
}
