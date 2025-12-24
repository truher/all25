package org.team100.lib.geometry;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.spline.PoseWithCurvature;

/**
 * Various distance metrics and norms.
 */
public class Metrics {

    /**
     * Distance along the arc between the two poses (in either order) produced by a
     * constant twist.
     */
    public static double distanceM(Pose2d a, Pose2d b) {
        return translationalNorm(
                GeometryUtil.slog(
                        GeometryUtil.transformBy(GeometryUtil.inverse(a), b)));
    }

    /**
     * The norm of the translational part of the twist. Note this does not match the
     * path length for nonzero omega.
     */
    public static double translationalNorm(Twist2d a) {
        return Math.hypot(a.dx, a.dy);
    }

    /* All components of the twist. */
    public static double l2Norm(Twist2d a) {
        return Math.sqrt(a.dx * a.dx + a.dy * a.dy + a.dtheta * a.dtheta);
    }

    /** All components of the twist */
    public static double l2Norm(Twist3d t) {
        Vector<N6> v = VecBuilder.fill(t.dx, t.dy, t.dz, t.rx, t.ry, t.rz);
        return v.norm();
    }

    /** Angle from a to b */
    public static double distanceRad(Rotation2d a, Rotation2d b) {
        return b.minus(a).getRadians();
    }

    /** Just translational velocity */
    public static double translationalNorm(ChassisSpeeds a) {
        // Common case (for tank drives) of dy == 0
        if (a.vyMetersPerSecond == 0.0)
            return Math.abs(a.vxMetersPerSecond);
        return Math.hypot(a.vxMetersPerSecond, a.vyMetersPerSecond);
    }

    /**
     * Translational part of the twist from a to b.
     * 
     * https://vnav.mit.edu/material/04-05-LieGroups-notes.pdf
     */
    public static double distance(PoseWithCurvature a, PoseWithCurvature b) {
        return translationalNorm(
                GeometryUtil.slog(GeometryUtil.transformBy(GeometryUtil.inverse(a.poseMeters), b.poseMeters)));
    }

    /**
     * Double-geodesic combines the angular distance with the translational
     * distance, weighting 1 radian equal to 1 meter.
     * 
     * This is not the geodesic distance, which is zero for spin-in-place. It's just
     * the L2 norm for all three dimensions.
     * 
     * TODO: adjustable weights
     * 
     * Note the Chirikjian paper below suggests using mass and inertia for weighting
     * 
     * I thought this would be a good idea for generalizing path distances to SE(2),
     * but now I'm not sure.
     * 
     * @see https://vnav.mit.edu/material/04-05-LieGroups-notes.pdf
     * @see https://rpk.lcsr.jhu.edu/wp-content/uploads/2017/08/Partial-Bi-Invariance-of-SE3-Metrics1.pdf
     */
    public static double doubleGeodesicDistance(Pose2d a, Pose2d b) {
        Translation2d tDiff = a.getTranslation().minus(b.getTranslation());
        double tSqDist = GeometryUtil.dot(tDiff, tDiff);
        double aDiff = a.getRotation().minus(b.getRotation()).getRadians();
        if (GeometryUtil.DEBUG)
            System.out.printf("double geodesic distance t %f a %f\n", tSqDist, aDiff * aDiff);
        return Math.sqrt(aDiff * aDiff + tSqDist);
    }

    /** Double geodesic distance between the pose components */
    public static double doubleGeodesicDistance(WaypointSE2 a, WaypointSE2 b) {
        return doubleGeodesicDistance(a.pose(), b.pose());
    }

    /** Double geodesic distance between the pose components */
    public static double doubleGeodesicDistance(Pose2dWithMotion a, Pose2dWithMotion b) {
        return doubleGeodesicDistance(a.getPose(), b.getPose());
    }

}
