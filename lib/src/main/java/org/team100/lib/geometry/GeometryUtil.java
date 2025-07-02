package org.team100.lib.geometry;

import java.util.Optional;

import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.geometry.Twist3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N6;
import edu.wpi.first.math.spline.PoseWithCurvature;

/**
 * Lots of utility functions.
 */
public class GeometryUtil {
    private static final boolean DEBUG = false;

    private GeometryUtil() {
    }

    /** Return a projected onto the direction of b, retaining the omega of a */
    public static ChassisSpeeds project(ChassisSpeeds a, ChassisSpeeds b) {
        double norm = norm(b);
        if (norm < 1e-9) {
            // there's no target course, bail out
            return a;
        }
        double scale = dot(a, b) / (norm * norm);
        if (DEBUG)
            Util.printf("project() scale %.8f\n", scale);
        return new ChassisSpeeds(
                b.vx * scale,
                b.vy * scale,
                a.omega);
    }

    public static double dot(ChassisSpeeds a, ChassisSpeeds b) {
        return a.vx * b.vx + a.vy * b.vy;
    }

    public static double dot(Translation2d a, Translation2d b) {
        return a.getX() * b.getX() + a.getY() * b.getY();
    }

    public static double dot(Translation2d a, FieldRelativeVelocity b) {
        return a.getX() * b.x() + a.getY() * b.y();
    }

    public static Twist2d discretize(ChassisSpeeds continuous, double dt) {
        ChassisSpeeds speeds = continuous.discretize(dt);
        return new Twist2d(
                speeds.vx * dt,
                speeds.vy * dt,
                speeds.omega * dt);
    }

    public static Twist2d scale(Twist2d twist, double scale) {
        return new Twist2d(twist.dx * scale, twist.dy * scale, twist.dtheta * scale);
    }

    public static FieldRelativeVelocity scale(FieldRelativeVelocity v, double scale) {
        return new FieldRelativeVelocity(v.x() * scale, v.y() * scale, v.theta() * scale);
    }

    public static Pose2d transformBy(Pose2d a, Pose2d b) {
        return a.transformBy(new Transform2d(b.getTranslation(), b.getRotation()));
    }

    public static Pose2d inverse(Pose2d a) {
        Rotation2d rotation_inverted = a.getRotation().unaryMinus();
        return new Pose2d(a.getTranslation().unaryMinus().rotateBy(rotation_inverted), rotation_inverted);
    }

    public static Twist2d slog(final Pose2d p) {
        return Pose2d.kZero.log(p);
    }

    public static Twist3d slog(final Pose3d p) {
        return Pose3d.kZero.log(p);
    }

    public static Pose2d sexp(final Twist2d delta) {
        return Pose2d.kZero.exp(delta);
    }

    public static Pose2d fromRotation(final Rotation2d rotation) {
        return new Pose2d(new Translation2d(), rotation);
    }

    public static Pose2d fromTranslation(final Translation2d translation) {
        return new Pose2d(translation, Rotation2d.kZero);
    }

    public static Rotation2d fromRadians(double angle_radians) {
        return new Rotation2d(angle_radians);
    }

    public static Rotation2d fromDegrees(double angle_degrees) {
        return fromRadians(Math.toRadians(angle_degrees));
    }

    public static double WrapRadians(double radians) {
        final double k2Pi = 2.0 * Math.PI;
        radians = radians % k2Pi;
        radians = (radians + k2Pi) % k2Pi;
        if (radians > Math.PI)
            radians -= k2Pi;
        return radians;
    }

    /**
     * Rotations must be identical, translation is allowed along the rotational
     * direction but not any other direction.
     */
    public static boolean isColinear(Pose2d a, final Pose2d other) {
        if (!GeometryUtil.isParallel(a.getRotation(), other.getRotation()))
            return false;
        final Twist2d twist = slog(transformBy(inverse(a), other));
        return (Math.abs(twist.dy - 0.0) <= 1e-12
                && Math.abs(twist.dtheta - 0.0) <= 1e-12);
    }

    // note parallel also means antiparallel.
    public static boolean isParallel(Rotation2d a, Rotation2d b) {
        return Math.abs(a.getRadians() - b.getRadians()) <= 1e-12
                || Math.abs(a.getRadians() - WrapRadians(b.getRadians() + Math.PI)) <= 1e-12;
    }

    /**
     * The norm of the translational part of the twist. Note this does not match the
     * path length for nonzero omega.
     */
    public static double norm(Twist2d a) {
        // Common case of dy == 0
        if (a.dy == 0.0)
            return Math.abs(a.dx);
        return Math.hypot(a.dx, a.dy);
    }

    public static double norm(Twist3d t) {
        Vector<N6> v = VecBuilder.fill(t.dx, t.dy, t.dz, t.rx, t.ry, t.rz);
        return v.norm();
    }

    public static double norm(ChassisSpeeds a) {
        // Common case of dy == 0
        if (a.vy == 0.0)
            return Math.abs(a.vx);
        return Math.hypot(a.vx, a.vy);
    }

    public static boolean near(ChassisSpeeds a, ChassisSpeeds b) {
        return MathUtil.isNear(a.vx, b.vx, 1e-6)
                && MathUtil.isNear(a.vy, b.vy, 1e-6)
                && MathUtil.isNear(a.omega, b.omega, 1e-6);
    }

    public static Rotation2d flip(Rotation2d a) {
        return new Rotation2d(MathUtil.angleModulus(a.getRadians() + Math.PI));
    }

    public static double distance(Rotation2d a, final Rotation2d other) {
        return a.unaryMinus().rotateBy(other).getRadians();
    }

    public static Rotation2d interpolate2(Rotation2d a, final Rotation2d b, double x) {
        if (x <= 0.0) {
            return a;
        } else if (x >= 1.0) {
            return b;
        }
        double angle_diff = a.unaryMinus().rotateBy(b).getRadians();
        return a.rotateBy(Rotation2d.fromRadians(angle_diff * x));
    }

    /**
     * Interpolate between a and b, treating each dimension separately. This will
     * make straight lines, whereas the WPI Pose3d interpolator will make
     * constant-twist curves.
     */
    public static Pose3d interpolate(Pose3d a, Pose3d b, double x) {
        if (x <= 0.0) {
            return a;
        } else if (x >= 1.0) {
            return b;
        }
        Translation3d aT = a.getTranslation();
        Translation3d bT = b.getTranslation();
        Rotation3d aR = a.getRotation();
        Rotation3d bR = b.getRotation();
        // each translation axis is interpolated separately
        Translation3d lerpT = aT.interpolate(bT, x);
        // Rotation3d lerpR = aR.interpolate(bR, x);
        Rotation3d lerpR = interpolate(aR, bR, x);
        return new Pose3d(lerpT, lerpR);
    }

    /**
     * Interpolate between a and b, treating each dimension separately. This will
     * make straight lines, whereas the WPI Rotation3d interpolator uses quaternion
     * composition.
     */
    public static Rotation3d interpolate(Rotation3d a, Rotation3d b, double x) {
        if (x <= 0.0) {
            return a;
        } else if (x >= 1.0) {
            return b;
        }
        return new Rotation3d(
                MathUtil.interpolate(a.getX(), b.getX(), x),
                MathUtil.interpolate(a.getY(), b.getY(), x),
                MathUtil.interpolate(a.getZ(), b.getZ(), x));
    }

    public static double distance(PoseWithCurvature a, PoseWithCurvature b) {
        // this is not used
        return norm(slog(transformBy(inverse(a.pose), b.pose)));
    }

    /**
     * Distance along the arc between the two poses (in either order) produced by a
     * constant twist.
     */
    public static double distance(Pose2d a, Pose2d b) {
        return norm(slog(transformBy(inverse(a), b)));
    }

    public static double distance(Translation2d a, Translation2d b) {
        return inverse(a).plus(b).getNorm();
    }

    public static Translation2d inverse(Translation2d a) {
        return new Translation2d(-a.getX(), -a.getY());
    }

    public static PoseWithCurvature interpolate2(PoseWithCurvature a, final PoseWithCurvature other, double x) {
        Pose2d interpolatedPose = a.pose.interpolate(other.pose, x);
        double interpolatedCurvature = MathUtil.interpolate(a.curvature, other.curvature, x);
        return new PoseWithCurvature(interpolatedPose, interpolatedCurvature);
    }

    public static Twist2d interpolate(Twist2d a, Twist2d b, double x) {
        return new Twist2d(MathUtil.interpolate(a.dx, b.dx, x),
                MathUtil.interpolate(a.dy, b.dy, x),
                MathUtil.interpolate(a.dtheta, b.dtheta, x));
    }

    public static boolean poseWithCurvatureEquals(PoseWithCurvature a, PoseWithCurvature b) {
        boolean poseEqual = a.pose.equals(b.pose);
        if (!poseEqual) {
            return false;
        }
        return Math.abs(a.curvature - b.curvature) <= 1e-12;
    }

    /** direction of the translational part of the twist */
    public static Optional<Rotation2d> getCourse(Twist2d t) {
        if (norm(t) > 1e-12) {
            return Optional.of(new Rotation2d(t.dx, t.dy));
        } else {
            return Optional.empty();
        }
    }

    /** robot-relative course */
    public static Optional<Rotation2d> getCourse(ChassisSpeeds t) {
        if (norm(t) > 1e-12) {
            return Optional.of(new Rotation2d(t.vx, t.vy));
        } else {
            return Optional.empty();
        }
    }

    public static boolean isZero(ChassisSpeeds x) {
        return Math.abs(x.vx) < 1E-9
                && Math.abs(x.vy) < 1E-9
                && Math.abs(x.omega) < 1E-9;
    }

    /**
     * Transform the camera-coordinates translation to NWU coordinates. Note an
     * additional transform will be required to account for the camera offset
     * relative to the robot.
     * 
     * @param zForward translation in camera coordinates, z-forward
     * @return translation in WPI coordinates, x-forward.
     */
    public static Translation3d zForwardToXForward(Translation3d zForward) {
        return new Translation3d(zForward.getZ(), -zForward.getX(), -zForward.getY());
    }

    /**
     * Transform the NWU coordinates translation to camera-coordinates.
     * 
     * @param xForward translation in WPI coordinates, x-forward
     * @return translation in camera coordinates, z-forward.
     */
    public static Translation3d xForwardToZForward(Translation3d xForward) {
        return new Translation3d(-xForward.getY(), -xForward.getZ(), xForward.getX());
    }

    /**
     * Transform the NWU coordinates rotation to camera-coordinates.
     * 
     * @param xForward rotation in WPI coordinates, x-forward
     * @return rotation in camera coordinates, z-forward.
     */
    public static Rotation3d xForwardToZForward(Rotation3d xForward) {
        Quaternion q = xForward.getQuaternion();
        Quaternion q2 = new Quaternion(q.getW(), -q.getY(), -q.getZ(), q.getX());
        return new Rotation3d(q2);
    }

    /**
     * Transform the camera-coordinates rotation to NWU coordinates. Note an
     * additional transform will be required to account for the camera orientation
     * relative to the robot.
     * 
     * @param zforward rotation in camera coordinates, z-forward
     * @return rotation in WPI coordinates x-forward
     */
    public static Rotation3d zForwardToXForward(Rotation3d zforward) {
        Quaternion q = zforward.getQuaternion();
        Quaternion q2 = new Quaternion(q.getW(), q.getZ(), -q.getX(), -q.getY());
        return new Rotation3d(q2);
    }

    public static Vector<N3> toVec(Twist2d twist) {
        return VecBuilder.fill(twist.dx, twist.dy, twist.dtheta);
    }

    public static Vector<N6> toVec(Twist3d twist) {
        return VecBuilder.fill(twist.dx, twist.dy, twist.dz, twist.rx, twist.ry, twist.rz);
    }
}
