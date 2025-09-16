package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;

/** Trivial constraint for testing. */
public class ConstantConstraint implements TimingConstraint {
    private final double m_maxVelocity;
    private final double m_maxAccel;

    public ConstantConstraint(double maxV, double maxA) {
        m_maxVelocity = maxV;
        m_maxAccel = maxA;
    }

    public ConstantConstraint(double vScale, double aScale, SwerveKinodynamics limits) {
        this(vScale * limits.getMaxDriveVelocityM_S(), aScale * limits.getMaxDriveAccelerationM_S2());
    }

    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
        return new NonNegativeDouble(m_maxVelocity);
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(Pose2dWithMotion state, double velocityM_S) {
        return new MinMaxAcceleration(-m_maxAccel, m_maxAccel);
    }

}
