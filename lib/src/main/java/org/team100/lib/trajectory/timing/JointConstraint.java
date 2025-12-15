package org.team100.lib.trajectory.timing;

import org.team100.lib.geometry.AccelerationSE2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.geometry.Pose2dWithDirection;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.prr.AnalyticalJacobian;
import org.team100.lib.subsystems.prr.EAWConfig;
import org.team100.lib.subsystems.prr.ElevatorArmWristKinematics;
import org.team100.lib.subsystems.prr.JointAccelerations;
import org.team100.lib.subsystems.prr.JointVelocities;

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
 * TODO: make this constraint actually work, I think it's broken.
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
        Pose2dWithDirection pose = state.getPose();
        // Velocity if translation speed were 1.0 m/s.
        VelocitySE2 v = new VelocitySE2(
                state.getCourse().getCos(),
                state.getCourse().getSin(),
                state.getHeadingRateRad_M());
        ModelR3 m = new ModelR3(pose.pose(), v);

        // corresponding vector in joint space
        JointVelocities qdot = m_j.inverse(m);

        // as a fraction of maxima
        double elevatorScale = Math.abs(qdot.elevator() / m_maxJv.elevator());
        double shoulderScale = Math.abs(qdot.shoulder() / m_maxJv.shoulder());
        double wristScale = Math.abs(qdot.wrist() / m_maxJv.wrist());

        double maxScale = Math.max(elevatorScale, Math.max(shoulderScale, wristScale));

        // scale qdot to the nearest maximum
        JointVelocities maxQdotInMotionDirection = qdot.times(1 / maxScale);

        EAWConfig q = m_k.inverse(pose.pose());

        VelocitySE2 maxV = m_j.forward(q, maxQdotInMotionDirection);
        double norm = maxV.norm();
        if (Double.isNaN(norm))
            return new NonNegativeDouble(0);
        return new NonNegativeDouble(norm);
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(
            Pose2dWithMotion state, double velocityM_S) {
        Pose2d pose = state.getPose().pose();
        Rotation2d course2 = state.getPose().course().toRotation();

        double c = course2.getCos();
        double s = course2.getSin();
        double r = state.getHeadingRateRad_M();
        double vx = velocityM_S * s;
        double vy = velocityM_S * c;
        double omega = velocityM_S * r;

        // actual cartesian velocity
        VelocitySE2 v = new VelocitySE2(vx, vy, omega);

        EAWConfig q = m_k.inverse(pose);
        // actual qdot
        JointVelocities qdot = m_j.inverse(new ModelR3(pose, v));

        // find accel in motion
        AccelerationSE2 unitA = new AccelerationSE2(c, s, r);
        ControlR3 sc = new ControlR3(pose, v, unitA);
        // corresponding a vector in joint space
        JointAccelerations qddot = m_j.inverseA(sc);

        // as a fraction of maxima
        double elevatorScale = Math.abs(qddot.elevator() / m_maxJa.elevator());
        double shoulderScale = Math.abs(qddot.shoulder() / m_maxJa.shoulder());
        double wristScale = Math.abs(qddot.wrist() / m_maxJa.wrist());

        double maxScale = Math.max(elevatorScale, Math.max(shoulderScale, wristScale));

        // scale qddot to the nearest maximum
        JointAccelerations maxQddotInMotionDirection = qddot.times(1 / maxScale);

        AccelerationSE2 fa = m_j.forwardA(q, qdot, maxQddotInMotionDirection);

        double norm = fa.norm();
        if (Double.isNaN(norm))
            return new MinMaxAcceleration(0, 0);
        return new MinMaxAcceleration(-1.0 * norm, 1.0 * norm);
    }

}
