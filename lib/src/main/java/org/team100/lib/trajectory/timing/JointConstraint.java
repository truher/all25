package org.team100.lib.trajectory.timing;

import java.util.Optional;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.motion.Config;
import org.team100.lib.motion.drivetrain.state.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.motion.kinematics.AnalyticalJacobian;
import org.team100.lib.motion.kinematics.ElevatorArmWristKinematics;
import org.team100.lib.motion.kinematics.JointAccelerations;
import org.team100.lib.motion.kinematics.JointVelocities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * For cartesian trajectories executed by non-cartesian mechanisms: each joint
 * has its own velocity and acceleration constraints, which are converted to the
 * cartesian TimingConstraint via the Jacobian.
 * 
 * Acceleration constraints here are constant.
 * 
 * TODO: make acceleration constraint depend on position, to account for
 * gravity, because it's really a motor torque constraint, not an acceleration
 * constraint per se.
 */
public class JointConstraint implements TimingConstraint {
    private final ElevatorArmWristKinematics m_k;
    private final AnalyticalJacobian m_j;
    private final JointVelocities m_maxJv;
    private final JointAccelerations m_maxJa;

    public JointConstraint(
            ElevatorArmWristKinematics k,
            AnalyticalJacobian j,
            JointVelocities maxJv,
            JointAccelerations maxJa) {
        m_k = k;
        m_j = j;
        m_maxJv = maxJv;
        m_maxJa = maxJa;
    }

    @Override
    public NonNegativeDouble getMaxVelocity(Pose2dWithMotion state) {
        Pose2d pose = state.getPose();
        Optional<Rotation2d> course2 = state.getCourse();

        double c = course2.map(Rotation2d::getCos).orElse(0.0);
        double s = course2.map(Rotation2d::getSin).orElse(0.0);

        Config q = m_k.inverse(pose);

        // find the cartesian velocity for the max joint velocity
        // TODO: this is wrong, it uses *one* joint velocity;
        // what we want is to find the ellipsoid in cartesian
        // space corresponding to the sphere in joint space.
        FieldRelativeVelocity v = m_j.forward(q, m_maxJv);

        // find the component in the direction of motion
        double vel = v.x() * c + v.y() * s;

        return new NonNegativeDouble(Math.abs(vel));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(
            Pose2dWithMotion state, double velocityM_S) {
        Pose2d pose = state.getPose();
        Optional<Rotation2d> course2 = state.getCourse();

        double c = course2.map(Rotation2d::getCos).orElse(0.0);
        double s = course2.map(Rotation2d::getSin).orElse(0.0);
        double r = state.getHeadingRate();
        double vx = velocityM_S * s;
        double vy = velocityM_S * c;
        double omega = velocityM_S * r;
        FieldRelativeVelocity v = new FieldRelativeVelocity(vx, vy, omega);

        Config q = m_k.inverse(pose);
        JointVelocities qdot = m_j.inverse(new SwerveModel(pose, v));

        // find the cartesian accel for the max joint accel
        // TODO: this is wrong, it uses *one* joint velocity;
        // what we want is to find the ellipsoid in cartesian
        // space corresponding to the sphere in joint space.
        FieldRelativeAcceleration fa = m_j.forwardA(q, qdot, m_maxJa);

        // find the component in the direction of motion
        double acc = fa.x() * c + fa.y() * s;

        return new MinMaxAcceleration(-1.0 * Math.abs(acc), 1.0 * Math.abs(acc));
    }

}
