package org.team100.lib.commands.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldConstants {

    public enum FieldSector {
        AB, CD, EF, GH, IJ, KL;
    }

    public enum ReefPoint {
        A, B, C, D, E, F, G, H, I, J, K, L
    }

    public enum ReefDestination {
        LEFT, RIGHT, CENTER
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

    public static FieldConstants.FieldSector getSector(Pose2d pose) {
        Translation2d target = FieldConstants.getReefCenter().minus(pose.getTranslation());
        Rotation2d targetAngle = target.getAngle();
        double angle = targetAngle.getDegrees();

        if (angle >= -30 && angle <= 30) {
            return FieldSector.AB;
        } else if (angle >= -90 && angle < -30) {
            return FieldSector.KL;
        } else if (angle >= -150 && angle <= -90) {
            return FieldSector.IJ;
        } else if (angle >= 150 || angle <= -150) {
            return FieldSector.GH;
        } else if (angle >= 90 && angle <= 150) {
            return FieldSector.EF;
        } else if (angle >= 30 && angle <= 90) {
            return FieldSector.CD;
        } else {
            return FieldSector.AB;
        }
    }

    public static Rotation2d getSectorAngle(FieldSector sector) {
        switch (sector) {
            case AB:
                return Rotation2d.fromDegrees(180);
            case CD:
                return Rotation2d.fromDegrees(-120);
            case EF:
                return Rotation2d.fromDegrees(-60);
            case GH:
                return Rotation2d.fromDegrees(0);
            case IJ:
                return Rotation2d.fromDegrees(60);
            case KL:
                return Rotation2d.fromDegrees(120);
            default:
                return Rotation2d.fromDegrees(0);
        }
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

    public static double getReefOffset() {
        return 0.1645; // 0.1524
    }

    public static Rotation2d calculateDeltaSpline(Rotation2d originalRotation, Rotation2d deltaRotation, double scale) {
        Translation2d originalTranslation = new Translation2d(1, originalRotation);
        Translation2d deltaTranslation = new Translation2d(scale, deltaRotation);
        Translation2d newTranslation = originalTranslation.plus(deltaTranslation);
        return newTranslation.getAngle();
    }

    public static Translation2d getOrbitDestination(FieldSector destinationSector, ReefDestination destinationPoint,
            double radius) {
        Translation2d reefCenter = getReefCenter();
        Rotation2d sectorAngle = getSectorAngle(destinationSector);

        double x = reefCenter.getX() + radius * sectorAngle.getCos();
        double y = reefCenter.getY() + radius * sectorAngle.getSin();

        double dx = 0;
        double dy = 0;

        Rotation2d newRotation = Rotation2d.fromDegrees(sectorAngle.getDegrees() + 90);
        dy = (getReefOffset() * Math.sin(newRotation.getRadians()));
        dx = (getReefOffset() * Math.cos(newRotation.getRadians()));

        switch (destinationPoint) {
            case RIGHT:
                return new Translation2d(x += dx, y += dy);
            case LEFT:
                return new Translation2d(x -= dx, y -= dy);
            case CENTER:
                return new Translation2d(x, y);
        }
        return new Translation2d(x, y);

    }
}
