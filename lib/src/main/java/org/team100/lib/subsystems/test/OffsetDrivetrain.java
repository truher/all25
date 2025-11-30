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

/**
 * Demo of offset control, without actually changing any control
 * classes.
 * 
 * The controlled state is the "toolpoint" of the robot.
 * 
 * The drivetrain is the delegate, and its velocity commands are
 * derived from the toolpoint velocities using a fixed offset.
 * 
 * Some of the toolpoint desired velocity is perpendicular to the offset, and so
 * by adding a rotation to the delegate, we can move the toolpoint in its
 * desired direction a bit faster, in exchange for some theta error. This
 * essentially edits the output of the controller, so we can leave the
 * controller alone.
 */
public class OffsetDrivetrain implements VelocitySubsystemR3 {
    /**
     * How much of the perpendicular speed to mix in. This interacts with the
     * controller "P" values, so should be tuned together with them.
     */
    private static final double OMEGA_MIXER = 2.0;
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
        // the component of the cartesian part that tries to spin
        // the delegate
        // adding some of this will make the toolpoint move more rapidly
        // towards the cartesian goal, while injecting theta error.
        GlobalVelocityR3 perpendicularOmega = omega(r(m_offset), velocity(setpoint));

        // the component of the rotation part that tries to move the
        // delegate in x and y
        // respecting 100% of this velocity will keep the toolpoint
        // where it wants to go (if the delegate responds perfectly)
        GlobalVelocityR3 tangentialVelocity = tangentialVelocity(omega(setpoint), r(m_offset.unaryMinus()));

        m_delegate.setVelocity(setpoint
                .plus(tangentialVelocity)
                .plus(perpendicularOmega.times(OMEGA_MIXER)));
    }

    @Override
    public void stop() {
        m_delegate.stop();
    }

    /**
     * The perpendicular component of v across r, as an angular velocity.
     * 
     * omega = (r \cross v) / r^2
     * 
     * Cartesian components are always zero.
     */
    private GlobalVelocityR3 omega(Vector<N3> r, Vector<N3> v) {
        return GlobalVelocityR3.fromVector(
                Vector.cross(r, v).div(r.norm() * r.norm()));
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
    private GlobalVelocityR3 toolpointVelocity() {
        GlobalVelocityR3 delegateVelocity = m_delegate.getState().velocity();
        return delegateVelocity.plus(
                tangentialVelocity(omega(delegateVelocity), r(m_offset)));
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

    /**
     * Cartesian component of velocity.
     */
    private Vector<N3> velocity(GlobalVelocityR3 v) {
        return v.vVector();
    }

    /**
     * Omega component of the velocity
     */
    private Vector<N3> omega(GlobalVelocityR3 v) {
        return v.omegaVector();
    }

    /**
     * Computes the cartesian velocity created by the rotational velocity omega,
     * through the radius r.
     * 
     * v = omega \cross r
     * 
     * Omega component is always zero.
     */
    private GlobalVelocityR3 tangentialVelocity(
            Vector<N3> omega, Vector<N3> r) {
        return GlobalVelocityR3.fromVector(
                Vector.cross(omega, r));
    }

}
