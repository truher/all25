package org.team100.lib.subsystems.test;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

public class OffsetDrivetrain implements VelocitySubsystemR3 {
    private final VelocitySubsystemR3 m_delegate;
    private final Translation2d m_offset;

    /**
     * @param delgate the real drivetrain
     * @param offset  from delegate to toolpoint
     */
    public OffsetDrivetrain(
            VelocitySubsystemR3 delegate, Translation2d offset) {
        m_delegate = delegate;
        m_offset = offset;
    }

    @Override
    public ModelR3 getState() {
        return new ModelR3(toolpointPose(), toolpointVelocity());
    }

    /**
     * Set delegate velocity from toolpoint velocity and offset.
     * r is from toolpoint to delegate, so invert offset.
     * 
     * @param setpoint toolpoint velocity
     */
    @Override
    public void setVelocity(GlobalVelocityR3 setpoint) {
        Translation2d inverseOffset = m_offset.unaryMinus();
        m_delegate.setVelocity(setpoint.plus(
                tangentialVelocity(omega(setpoint), r(inverseOffset))));
    }

    @Override
    public void stop() {
        m_delegate.stop();
    }

    /** Compute toolpoint pose from delegate pose and offset. */
    private Pose2d toolpointPose() {
        return m_delegate.getState().pose().transformBy(
                new Transform2d(m_offset, Rotation2d.kZero));
    }

    /** Compute toolpoint velocity from delegate velocity, pose, and offset. */
    private GlobalVelocityR3 toolpointVelocity() {
        GlobalVelocityR3 delegateVelocity = m_delegate.getState().velocity();
        return delegateVelocity.plus(
                tangentialVelocity(omega(delegateVelocity), r(m_offset)));
    }

    private Rotation2d delegateRotation() {
        return m_delegate.getState().rotation();
    }

    /** radial vector */
    private Vector<N3> r(Translation2d offset) {
        return GeometryUtil.toVec3(
                offset.rotateBy(delegateRotation()));
    }

    private Vector<N3> omega(GlobalVelocityR3 v) {
        return v.omegaVector();
    }

    /**
     * v = omega \cross r
     */
    private GlobalVelocityR3 tangentialVelocity(
            Vector<N3> omega, Vector<N3> r) {
        return GlobalVelocityR3.fromVector(
                Vector.cross(omega, r));
    }

}
