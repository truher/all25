package org.team100.lib.motion.kinematics;

import org.team100.lib.motion.Config;
import org.team100.lib.motion.drivetrain.state.GlobalSe2Acceleration;
import org.team100.lib.motion.drivetrain.state.GlobalSe2Velocity;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

/**
 * The PRR end-effector Jacobian is simple enough to write out. It's almost
 * identical to the center-of-mass Jacobian for the last link, used for
 * dynamics.
 * 
 * See doc/README.md
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

    /**
     * Forward velocity kinematics.
     * 
     * \dot{x} = J\dot{q}
     * 
     * See doc/README.md equation 4
     */
    public GlobalSe2Velocity forward(Config q, JointVelocities qdot) {
        Matrix<N3, N3> j = getJ(q);
        return GlobalSe2Velocity.fromVector(j.times(qdot.toVector()));
    }

    /**
     * Inverse velocity kinematics.
     * 
     * \dot{q} = J^{-1}\dot{x}
     * 
     * See README.md equation 5
     */
    public JointVelocities inverse(SwerveModel m) {
        Pose2d x = m.pose();
        GlobalSe2Velocity xdot = m.velocity();
        Config q = m_k.inverse(x);
        Matrix<N3, N3> Jinv = getJinv(q);
        return JointVelocities.fromVector(Jinv.times(xdot.toVector()));
    }

    /**
     * Forward acceleration kinematics.
     * 
     * \ddot{x} = \dot{J}\dot{q} + J\ddot{q}
     * 
     * See doc/README.md equation 6
     */
    public GlobalSe2Acceleration forwardA(
            Config q, JointVelocities qdot, JointAccelerations qddot) {
        Matrix<N3, N3> J = getJ(q);
        Matrix<N3, N3> Jdot = getJdot(q, qdot);
        return GlobalSe2Acceleration.fromVector(
                Jdot.times(qdot.toVector()).plus(J.times(qddot.toVector())));
    }

    /**
     * Inverse acceleration kinematics.
     * 
     * \ddot{q} = J^{-1}(\ddot{x} - \dot{J}J^{-1}\dot{x})
     * 
     * See doc/README.md equation 9
     */
    public JointAccelerations inverseA(SwerveControl m) {
        Pose2d x = m.pose();
        GlobalSe2Velocity xdot = m.velocity();
        GlobalSe2Acceleration xddot = m.acceleration();
        Config q = m_k.inverse(x);
        Matrix<N3, N3> Jinv = getJinv(q);
        JointVelocities qdot = JointVelocities.fromVector(Jinv.times(xdot.toVector()));
        Matrix<N3, N3> Jdot = getJdot(q, qdot);
        return JointAccelerations.fromVector(
                Jinv.times(
                        xddot.toVector().minus(
                                Jdot.times(Jinv.times(xdot.toVector())))));
    }

    /////////////////////////////////////////////////

    /**
     * End-effector Jacobian.
     * 
     * See doc/README.md equation 3
     */
    private Matrix<N3, N3> getJ(Config q) {
        double q2 = q.shoulderAngle();
        double q3 = q.wristAngle();
        double s2 = Math.sin(q2);
        double c2 = Math.cos(q2);
        double s23 = Math.sin(q2 + q3);
        double c23 = Math.cos(q2 + q3);
        Matrix<N3, N3> J = new Matrix<>(Nat.N3(), Nat.N3());
        J.set(0, 0, 1);
        J.set(0, 1, -l2 * s2 - l3 * s23);
        J.set(0, 2, -l3 * s23);
        J.set(1, 0, 0);
        J.set(1, 1, l2 * c2 + l3 * c23);
        J.set(1, 2, l3 * c23);
        J.set(2, 0, 0);
        J.set(2, 1, 1);
        J.set(2, 2, 1);
        return J;
    }

    /**
     * Time-derivative of the end-effector Jacobian.
     * 
     * See doc/README.md equation 7
     */
    private Matrix<N3, N3> getJdot(Config q, JointVelocities qdot) {
        double q2 = q.shoulderAngle();
        double q3 = q.wristAngle();
        double s2 = Math.sin(q2);
        double c2 = Math.cos(q2);
        double s23 = Math.sin(q2 + q3);
        double c23 = Math.cos(q2 + q3);
        double q2dot = qdot.shoulder();
        double q3dot = qdot.wrist();
        Matrix<N3, N3> Jdot = new Matrix<>(Nat.N3(), Nat.N3());
        Jdot.set(0, 0, 0);
        Jdot.set(0, 1, -l2 * c2 * q2dot - l3 * c23 * (q2dot + q3dot));
        Jdot.set(0, 2, -l3 * c23 * (q2dot + q3dot));
        Jdot.set(1, 0, 0);
        Jdot.set(1, 1, -l2 * s2 * q2dot - l3 * s23 * (q2dot + q3dot));
        Jdot.set(1, 2, -l3 * s23 * (q2dot + q3dot));
        Jdot.set(2, 0, 0);
        Jdot.set(2, 1, 0);
        Jdot.set(2, 2, 0);
        return Jdot;
    }

    /**
     * Inverse Jacobian, or zero if singular.
     */
    private Matrix<N3, N3> getJinv(Config q) {
        Matrix<N3, N3> J = getJ(q);
        if (Math.abs(J.det()) < 1e-3) {
            // Don't try to invert if it's not possible.
            // a zero inverse determinant will result in zero speed,
            // which is the safe thing.
            System.out.printf("WARNING: zero jacobian for config %s\n", q.toString());
            return new Matrix<>(Nat.N3(), Nat.N3());
        }
        return J.inv();
    }
}
