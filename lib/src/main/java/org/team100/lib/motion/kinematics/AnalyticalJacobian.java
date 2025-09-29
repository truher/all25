package org.team100.lib.motion.kinematics;

import org.team100.lib.motion.Config;
import org.team100.lib.motion.drivetrain.state.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

/**
 * The PRR end-effector Jacobian is simple enough to write out. It's almost
 * identical to the center-of-mass Jacobian for the last link, used for
 * dynamics.
 */
public class AnalyticalJacobian {
    private final ElevatorArmWristKinematics m_k;
    // notation from PRRDynamics.
    private final double l2;
    private final double l3;

    public AnalyticalJacobian(ElevatorArmWristKinematics k) {
        m_k = k;
        l2 = k.getArmLength();
        l3 = k.getManipulatorLength();
    }

    public FieldRelativeVelocity forward(Config c, JointVelocities v) {
        Matrix<N3, N3> j = getJ(c);
        return FieldRelativeVelocity.fromVector(j.times(v.toVector()));
    }

    public FieldRelativeAcceleration forwardA(Config c, JointVelocities v, JointAccelerations a) {
        return null;
    }

    private Matrix<N3, N3> getJ(Config c) {
        double s2 = Math.sin(c.shoulderAngle());
        double c2 = Math.cos(c.shoulderAngle());
        double s23 = Math.sin(c.shoulderAngle() + c.wristAngle());
        double c23 = Math.cos(c.shoulderAngle() + c.wristAngle());
        Matrix<N3, N3> j = new Matrix<>(Nat.N3(), Nat.N3());
        j.set(0, 0, 1);
        j.set(0, 1, -l2 * s2 - l3 * s23);
        j.set(0, 2, -l3 * s23);
        j.set(1, 0, 0);
        j.set(1, 1, l2 * c2 + l3 * c23);
        j.set(1, 2, l3 * c23);
        j.set(2, 0, 0);
        j.set(2, 1, 1);
        j.set(2, 2, 1);
        return j;
    }

    public JointVelocities inverse(SwerveModel swerveModel) {
        Pose2d p = swerveModel.pose();
        FieldRelativeVelocity v = swerveModel.velocity();
        Config c = m_k.inverse(p);
        Matrix<N3, N3> j = getJ(c);
        if (Math.abs(j.det()) < 1e-3) {
            // Don't try to invert if it's not possible.
            // The safe thing is to stop.
            return new JointVelocities(0, 0, 0);
        }
        return JointVelocities.fromVector(j.inv().times(v.toVector()));
    }

    public JointAccelerations inverseA(SwerveModel swerveModel) {
        return null;
    }

}
