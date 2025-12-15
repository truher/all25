package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithDirection;
import org.team100.lib.geometry.Pose2dWithMotion;
import org.team100.lib.subsystems.prr.AnalyticalJacobian;
import org.team100.lib.subsystems.prr.ElevatorArmWristKinematics;
import org.team100.lib.subsystems.prr.JointAccelerations;
import org.team100.lib.subsystems.prr.JointVelocities;

public class JointConstraintTest {
    private static final double DELTA = 0.001;

    @Test
    void test1() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);
        JointVelocities maxJv = new JointVelocities(1, 1, 1);
        JointAccelerations maxJa = new JointAccelerations(1, 1, 1);
        JointConstraint jc = new JointConstraint(k, j, maxJv, maxJa);
        // motion +x, limiter is elevator.
        Pose2dWithMotion state = new Pose2dWithMotion(
                Pose2dWithDirection.make(3, 0, 0, 0), 0, 0, 0);
        assertEquals(1, jc.getMaxVelocity(state).getValue(), DELTA);
        assertEquals(-1, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(1, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

    @Test
    void test2() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);
        JointVelocities maxJv = new JointVelocities(1, 1, 1);
        JointAccelerations maxJa = new JointAccelerations(1, 1, 1);
        JointConstraint jc = new JointConstraint(k, j, maxJv, maxJa);
        // motion +y, shoulder and wrist have same constraint
        Pose2dWithMotion state = new Pose2dWithMotion(
                Pose2dWithDirection.make(3, 0, 0, Math.PI / 2), 0, 0, 0);
        assertEquals(2, jc.getMaxVelocity(state).getValue(), DELTA);
        assertEquals(-2, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(2, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

    @Test
    void test3() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);
        JointVelocities maxJv = new JointVelocities(1, 1, 1);
        JointAccelerations maxJa = new JointAccelerations(1, 1, 1);
        JointConstraint jc = new JointConstraint(k, j, maxJv, maxJa);
        // bent wrist, motion +x, bend doesn't matter.
        Pose2dWithMotion state = new Pose2dWithMotion(
                Pose2dWithDirection.make(2, 1, Math.PI / 2, 0), 0, 0, 0);
        assertEquals(1, jc.getMaxVelocity(state).getValue(), DELTA);
        assertEquals(-1, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(1, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

    @Test
    void test4() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);
        JointVelocities maxJv = new JointVelocities(1, 1, 1);
        JointAccelerations maxJa = new JointAccelerations(1, 1, 1);
        JointConstraint jc = new JointConstraint(k, j, maxJv, maxJa);
        // bent wrist, motion +y, shoulder is the limiter
        Pose2dWithMotion state = new Pose2dWithMotion(
                Pose2dWithDirection.make(2, 1, Math.PI / 2, Math.PI / 2), 0, 0, 0);
        assertEquals(2, jc.getMaxVelocity(state).getValue(), DELTA);
        assertEquals(-2, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(2, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

    @Test
    void test5() {
        ElevatorArmWristKinematics k = new ElevatorArmWristKinematics(2, 1);
        AnalyticalJacobian j = new AnalyticalJacobian(k);
        JointVelocities maxJv = new JointVelocities(1, 1, 1);
        JointAccelerations maxJa = new JointAccelerations(1, 1, 1);
        JointConstraint jc = new JointConstraint(k, j, maxJv, maxJa);
        // bent wrist and shoulder, arm at 45, motion +y,
        Pose2dWithMotion state = new Pose2dWithMotion(
                Pose2dWithDirection.make(2, 1 + Math.sqrt(2), Math.PI / 2, Math.PI / 2), 0, 0, 0);
        assertEquals(1, jc.getMaxVelocity(state).getValue(), DELTA);
        assertEquals(-1, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(1, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

}
