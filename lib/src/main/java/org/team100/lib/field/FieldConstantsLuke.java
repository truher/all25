package org.team100.lib.field;

import org.team100.lib.config.ElevatorUtil.ScoringLevel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstantsLuke {
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

    private static final Translation2d REEF_CENTER = new Translation2d(4.489, 4.026);


    public enum ReefPoint {
        A, B, C, D, E, F, G, H, I, J, K, L, AB, CD, EF, GH, IJ, KL, NONE;

        /** Direction away from each reef face. */
        Rotation2d angle() {
            return switch (this) {
                case A, B, AB -> Rotation2d.fromDegrees(0);
                case C, D, CD -> Rotation2d.fromDegrees(60);
                case E, F, EF -> Rotation2d.fromDegrees(120);
                case G, H, GH -> Rotation2d.fromDegrees(180);
                case I, J, IJ -> Rotation2d.fromDegrees(-120);
                case K, L, KL -> Rotation2d.fromDegrees(-60);
                case NONE -> Rotation2d.kZero;
                default -> throw new IllegalArgumentException();
            };
        }
    }

    public static Pose2d makeGoal(ScoringLevel level, ReefPoint point) {
        double radius = getRadius(point, level);
        Translation2d destination = getScoringDestination(point, radius);
        Rotation2d heading = point.angle().rotateBy(Rotation2d.k180deg);
        return new Pose2d(destination, heading);
    }

    static Translation2d getScoringDestination(ReefPoint point, double radius) {
        Rotation2d sectorAngle = point.angle();
        Translation2d spoke = new Translation2d(radius, sectorAngle);
        // center of the face of the reef
        Translation2d center = REEF_CENTER.plus(spoke);
        Rotation2d newRotation = sectorAngle.rotateBy(Rotation2d.kCCW_90deg);

        // end-effector is not in the center of robot y, it's offset a little.
        double endEffectorOffset = -0.193;
        // Half the distance between the poles, i.e. offset from the center, in meters.
        double reefOffset = 0.1645; // 0.1524
        Translation2d offsetLeft = new Translation2d(endEffectorOffset - reefOffset, newRotation);
        Translation2d offsetRight = new Translation2d(endEffectorOffset + reefOffset, newRotation);
        Translation2d offsetCenter = new Translation2d(endEffectorOffset, newRotation);

        return switch (point) {
            // left side
            case A, C, E, G, I, K -> center.plus(offsetLeft);
            // right side
            case B, D, F, H, J, L -> center.plus(offsetRight);
            case AB, CD, EF, GH, IJ, KL -> center.plus(offsetCenter);
            case NONE -> center;
            default -> throw new IllegalArgumentException();
        };
    }

    private static double getRadius(ReefPoint letter, ScoringLevel level) {
        return switch (level) {
            case L1 -> switch (letter) {
                case A, B, C, D, E, F, G, H, I, J, K, L -> 3;
                case AB, CD, EF, GH, IJ, KL -> 1.2;
                case NONE -> 3.0;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case L2 -> switch (letter) {
                case A, B, C, D, E, F, G, H, I, J, K, L -> 1.4;
                case AB, CD, EF, GH, IJ, KL -> 1.2;
                case NONE -> 3.0;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case L3 -> switch (letter) {
                case A -> 1.4;
                case AB -> 1.2;
                case B -> 1.4;

                case C -> 1.4;
                case CD -> 1.2;
                case D -> 1.4;

                case E -> 1.4;
                case EF -> 1.2;
                case F -> 1.4;

                case G -> 1.4;
                case GH -> 1.2;
                case H -> 1.4;

                case I -> 1.4;
                case IJ -> 1.2;
                case J -> 1.4;

                case K -> 1.4;
                case KL -> 1.2;
                case L -> 1.4;
                case NONE -> 3.0;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case L4 -> switch (letter) {
                case A -> 1.415;
                case AB -> 1.2;
                case B -> 1.415;

                case C -> 1.415;
                case CD -> 1.2;
                case D -> 1.415;

                case E -> 1.415;
                case EF -> 1.2;
                case F -> 1.415;

                case G -> 1.415;
                case GH -> 1.2;
                case H -> 1.465;

                case I -> 1.415;
                case IJ -> 1.2;
                case J -> 1.415;

                case K -> 1.415;
                case KL -> 1.2;
                case L -> 1.415;
                case NONE -> 3.0;
                default -> throw new IllegalArgumentException("invalid point");
            };
            case NONE -> 3.0;
            default -> throw new IllegalArgumentException("invalid level");
        };
    }
}
