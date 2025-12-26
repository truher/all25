package org.team100.lib.subsystems.test;

import org.team100.lib.geometry.GeometryUtil;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N3;

/**
 * Demo of offset control, without actually changing any control
 * classes.
 * 
 * The controlled state is the "toolpoint" of the robot.
 * 
 * The drivetrain is the delegate, and its velocity commands are
 * derived from the toolpoint velocities using a fixed offset.
 * 
 * This version of the offset drivetrain does not include boosting.
 */
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
     * @param nextV toolpoint velocity for the next timestep
     */
    @Override
    public void setVelocity(VelocitySE2 nextV) {
        // the component of the rotation part that tries to move the
        // delegate in x and y
        // respecting 100% of this velocity will keep the toolpoint
        // where it wants to go (if the delegate responds perfectly)
        VelocitySE2 tangentialVelocity = OffsetUtil.tangentialVelocity(
                OffsetUtil.omega(nextV), r(m_offset.unaryMinus()));

        m_delegate.setVelocity(nextV.plus(tangentialVelocity));
    }

    @Override
    public void stop() {
        m_delegate.stop();
    }

    /**
     * Computes toolpoint pose from delegate pose and offset.
     */
    private Pose2d toolpointPose() {
        return m_delegate.getState().pose().transformBy(
                new Transform2d(m_offset, Rotation2d.kZero));
    }

    /**
     * Computes toolpoint velocity from delegate velocity, pose, and offset.
     */
    private VelocitySE2 toolpointVelocity() {
        VelocitySE2 delegateVelocity = m_delegate.getState().velocity();
        return delegateVelocity.plus(
                OffsetUtil.tangentialVelocity(
                        OffsetUtil.omega(delegateVelocity), r(m_offset)));
    }

    private Rotation2d delegateRotation() {
        return m_delegate.getState().rotation();
    }

    /**
     * Vector form of the offset, rotated by the delegate pose rotation.
     */
    private Vector<N3> r(Translation2d offset) {
        return GeometryUtil.toVec3(
                offset.rotateBy(delegateRotation()));
    }

}
