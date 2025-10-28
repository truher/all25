package org.team100.lib.geometry;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Pose2dWithMotion is purely a spatial construct. It has no sense of time. So
 * rather than being in units of distance-per-time (or radians-per-time), the
 * denominator here is actually distance as well. Thus, it isn't really
 * meaningful to look at dx or dy directly - what is meaningful is:
 *
 * a) whether or not they are both zero (in which case this is a stationary or
 * turn-in-place motion)
 *
 * b) the angle formed by them, which is the direction of translation (in the
 * same coordinate frame as pose).
 *
 * Additionally, this means dtheta is in radians-per-distance if there is
 * translation, or radians-per-radian otherwise.
 */
public class MotionDirection {
    private final double m_dx;
    private final double m_dy;
    /** Radians per meter of motion. */
    private final double m_dtheta;

    /**
     * dx and dy are only useful to describe the direction.
     * dtheta is radians per *meter moved* not per second.
     */
    public MotionDirection(double dx, double dy, double dtheta) {
        m_dx = dx;
        m_dy = dy;
        m_dtheta = dtheta;
    }

    /**
     * Magnitude of the translational part of the motion.
     */
    double norm() {
        return Math.hypot(m_dx, m_dy);
    }

    /**
     * Direction of the translational part of the motion, or empty if motionless.
     */
    Optional<Rotation2d> course() {
        if (norm() > 1e-12)
            return Optional.of(new Rotation2d(m_dx, m_dy));
        return Optional.empty();
    }

    MotionDirection interpolate(MotionDirection other, double x) {
        return new MotionDirection(
                MathUtil.interpolate(m_dx, other.m_dx, x),
                MathUtil.interpolate(m_dy, other.m_dy, x),
                MathUtil.interpolate(m_dtheta, other.m_dtheta, x));
    }

    /** You probably don't want this. */
    public double dx() {
        return m_dx;
    }

    /** You probably don't want this. */
    public double dy() {
        return m_dy;
    }

    /** You probably don't want this. */
    public double dtheta() {
        return m_dtheta;
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj)
            return true;
        if (obj == null)
            return false;
        if (getClass() != obj.getClass())
            return false;
        MotionDirection other = (MotionDirection) obj;
        if (!MathUtil.isNear(m_dx, other.m_dx, 1e-6))
            return false;
        if (!MathUtil.isNear(m_dy, other.m_dy, 1e-6))
            return false;
        if (!MathUtil.isNear(m_dtheta, other.m_dtheta, 1e-6))
            return false;
        return true;
    }

    @Override
    public String toString() {
        String courseStr = course().map(
                (x) -> String.format("%5.3f", x.getRadians())).orElse("empty");
        return String.format("norm: %5.3f, course: %s", norm(), courseStr);
    }

}