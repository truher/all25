package org.team100.lib.trajectory.timing;

import java.util.Optional;

import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.geometry.Pose2dWithMotion.MotionDirection;
import org.team100.lib.motion.drivetrain.state.FieldRelativeAcceleration;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.motion.kinematics.AnalyticalJacobian;
import org.team100.lib.motion.kinematics.ElevatorArmWristKinematics;
import org.team100.lib.motion.kinematics.JointAccelerations;
import org.team100.lib.motion.kinematics.JointVelocities;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;

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
    /** don't barf if the max velocity seems to be zero; return this instead. */
    private static final double MIN_MAX_V = 1.0;
    private static final double MIN_MAX_A = 1.0;
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
        MotionDirection motion = state.getMotionDirection();
        // THIS IS NOT VELOCITY! It's kind of "spatial velocity".
        // you could also think of it as 1 m/s along the path
        FieldRelativeVelocity v = new FieldRelativeVelocity(
                motion.dx(), motion.dy(), motion.dtheta());
        SwerveModel m = new SwerveModel(pose, v);
        // required joint velocities to achieve 1 m/s
        JointVelocities jv = m_j.inverse(m);
        // required joint velocities as a fraction of maximum.
        Vector<N3> ratio = jv.div(m_maxJv);
        // maximum fraction
        double max = ratio.maxAbs();
        if (max < 1e-3) {
            return new NonNegativeDouble(MIN_MAX_V);
        }
        // the available max velocity is the inverse, e.g. if 1 m/s drives
        // a joint at 50% of its maximum, then the max v is 2.
        return new NonNegativeDouble(1.0 / max);
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
        // the acceleration constraint here is *along* the path, so
        // make a "unit" acceleration which does that.
        FieldRelativeAcceleration a = new FieldRelativeAcceleration(
                c, s, r);
        SwerveControl m = new SwerveControl(pose, v, a);
        // joint accel to achieve 1 m/s etc
        JointAccelerations ja = m_j.inverseA(m);
        // joint accel as a fraction of max
        Vector<N3> ratio = ja.div(m_maxJa);
        // max fraction
        double max = ratio.maxAbs();
        if (max < 1e-3) {
            max = MIN_MAX_A;
        }
        return new MinMaxAcceleration(-1.0 / max, 1.0 / max);
    }

}
