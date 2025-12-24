package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.subsystems.swerve.kinodynamics.SwerveKinodynamics;
import org.team100.lib.tuning.Mutable;

/**
 * Velocity limit based on curvature and the capsize limit (scaled).
 */
public class CapsizeAccelerationConstraint implements TimingConstraint {
    private final SwerveKinodynamics m_limits;
    private final Mutable m_scale;

    /**
     * Use the factory.
     * 
     * @param limits absolute maxima
     * @param scale  apply to the maximum capsize accel to get the actual
     *               constraint. this is useful to slow down trajectories in
     *               sharp curves, which makes odometry more accurate and reduces
     *               the effect of steering lag.
     */
    public CapsizeAccelerationConstraint(
            LoggerFactory parent,
            SwerveKinodynamics limits,
            double scale) {
        LoggerFactory log = parent.type(this);
        m_limits = limits;
        m_scale = new Mutable(log, "scale", scale);
    }

    /**
     * The centripetal acceleration as a function of linear speed and radius:
     * a = v^2 / r
     * so
     * v = sqrt(a * r)
     * If the curvature is zero, this will return infinity.
     */
    @Override
    public double maxV(final Pose2dWithMotion state) {
        double mMaxCentripetalAccel = m_limits.getMaxCapsizeAccelM_S2() * m_scale.getAsDouble();
        double radius = 1 / state.getCurvatureRad_M();
        // abs is used here to make sure sqrt is happy.
        return Math.sqrt(Math.abs(mMaxCentripetalAccel * radius));
    }

    @Override
    public double maxAccel(Pose2dWithMotion state, double velocity) {
        double alongsq = alongSq(state, velocity);
        if (alongsq < 0) {
            // too fast for the curvature, can't speed up
            return 0;
        }
        return Math.sqrt(alongsq);
    }

    @Override
    public double maxDecel(Pose2dWithMotion state, double velocity) {
        double alongsq = alongSq(state, velocity);
        if (alongsq < 0) {
            // too fast for the curvature, slowing down is ok
            return -m_limits.getMaxDriveDecelerationM_S2() * m_scale.getAsDouble();
        }
        return -Math.sqrt(alongsq);
    }

    /**
     * Acceleration has two components: along the path (which is what is returned
     * here), and across the path. Assuming the robot is "round," i.e. the capsize
     * limit is the same in all directions, then it's the *total* acceleration that
     * should be limited. This returns the along-the-path component of that total.
     * 
     * centripetal = v^2 / r
     * total^2 = centripetal^2 + along^2
     * so
     * along = sqrt(total^2 - v^4/r^2)
     */
    private double alongSq(Pose2dWithMotion state, double velocity) {
        double maxCentripetalAccel = m_limits.getMaxCapsizeAccelM_S2() * m_scale.getAsDouble();
        double radius = 1 / state.getCurvatureRad_M();
        double actualCentripetalAccel = velocity * velocity / radius;
        return maxCentripetalAccel * maxCentripetalAccel - actualCentripetalAccel * actualCentripetalAccel;
    }
}
