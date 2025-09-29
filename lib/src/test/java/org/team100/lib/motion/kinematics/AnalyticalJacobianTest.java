package org.team100.lib.motion.kinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.motion.Config;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.trajectory.Trajectory100;
import org.team100.lib.trajectory.TrajectoryPlanner;
import org.team100.lib.trajectory.timing.ConstantConstraint;
import org.team100.lib.trajectory.timing.TimedPose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Some tests from JacobianTest to verify correctness. */
public class AnalyticalJacobianTest {
    private static final double DELTA = 0.001;

    @Test
    void test05() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);

        Config c = new Config(1, 0, 0);
        Pose2d p = k.forward(c);

        // some example velocities
        // zero velocity
        SwerveModel v = new SwerveModel(p);

        JointVelocities jv = j.inverse(v);
        assertEquals(0, jv.elevator(), DELTA);
        assertEquals(0, jv.shoulder(), DELTA);
        assertEquals(0, jv.wrist(), DELTA);

        // +x
        v = new SwerveModel(p, new FieldRelativeVelocity(1, 0, 0));
        jv = j.inverse(v);
        assertEquals(1, jv.elevator(), DELTA);
        assertEquals(0, jv.shoulder(), DELTA);
        assertEquals(0, jv.wrist(), DELTA);

        // +y
        v = new SwerveModel(p, new FieldRelativeVelocity(0, 1, 0));
        jv = j.inverse(v);
        assertEquals(0, jv.elevator(), DELTA);
        assertEquals(0.5, jv.shoulder(), DELTA);
        assertEquals(-0.5, jv.wrist(), DELTA);

        // +theta
        v = new SwerveModel(p, new FieldRelativeVelocity(0, 0, 1));
        jv = j.inverse(v);
        assertEquals(0, jv.elevator(), DELTA);
        assertEquals(-0.5, jv.shoulder(), DELTA);
        assertEquals(1.5, jv.wrist(), DELTA);
    }

    @Test
    void testTrajectory() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);

        TrajectoryPlanner planner = new TrajectoryPlanner(List.of(new ConstantConstraint(1, 1)));
        Pose2d start = new Pose2d(1, -1, Rotation2d.kZero);
        Pose2d end = new Pose2d(2, 1, Rotation2d.k180deg);
        Trajectory100 t = planner.restToRest(start, end);
        double d = t.duration();
        double dt = d / 20;
        for (double time = 0; time < d; time += dt) {
            TimedPose tp = t.sample(time);
            SwerveModel sm = SwerveModel.fromTimedPose(tp);
            Pose2d p = sm.pose();
            FieldRelativeVelocity v = sm.velocity();
            Config c = k.inverse(p);
            JointVelocities jv = j.inverse(sm);
            System.out.printf(
                    "s (%5.2f) pose(%5.2f %5.2f %5.2f) conf(%5.2f %5.2f %5.2f) tv(%5.2f %5.2f %5.2f) jv(%5.2f %5.2f %5.2f)\n",
                    time,
                    p.getX(), p.getY(), p.getRotation().getRadians(),
                    c.shoulderHeight(), c.shoulderAngle(), c.wristAngle(),
                    v.x(), v.y(), v.theta(),
                    jv.elevator(), jv.shoulder(), jv.wrist());
        }
    }

    @Test
    void testInverseA() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);
        assertNotNull(j.inverseA(new SwerveModel()));
    }

    @Test
    void testForwardA() {
        final ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);
        assertNotNull(j.forwardA(
                new Config(0, 0, 0),
                new JointVelocities(0, 0, 0),
                new JointAccelerations(0, 0, 0)));
    }

}
