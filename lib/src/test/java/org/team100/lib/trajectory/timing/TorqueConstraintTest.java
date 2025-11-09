package org.team100.lib.trajectory.timing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.Pose2dWithMotion;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TorqueConstraintTest {
    private static final double DELTA = 0.001;

    @Test
    void testMotionless() {
        // 6 Nm at 1 m = 6 N; mass is 6 kg so a is 1
        TorqueConstraint jc = new TorqueConstraint(6);
        // motionless at (1,0,0)
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(1, 0, Rotation2d.kZero), 0, 0, 0, 0, 0);
        // motionless => worst case
        assertEquals(-1, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(1, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

    @Test
    void testRadial() {
        TorqueConstraint jc = new TorqueConstraint(6);
        // at (1,0,0), moving (1,0,0)
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(1, 0, Rotation2d.kZero), 1, 0, 0, 0, 0);
        // no tangential motion => no limit
        assertEquals(Double.NEGATIVE_INFINITY, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(Double.POSITIVE_INFINITY, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

    @Test
    void testTangential() {
        TorqueConstraint jc = new TorqueConstraint(6);
        // at (1,0,0), moving (0,1,0)
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(1, 0, Rotation2d.kZero), 0, 1, 0, 0, 0);
        // tangential motion at 1 m
        assertEquals(-1, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(1, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

    @Test
    void testInclined() {
        TorqueConstraint jc = new TorqueConstraint(6);
        // at (1,0,0), moving (1,1,0)
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(1, 0, Rotation2d.kZero), 1, 1, 0, 0, 0);
        // motion at 45 deg => higher limit
        assertEquals(-1.414, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(1.414, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

    @Test
    void testFar() {
        TorqueConstraint jc = new TorqueConstraint(6);
        // at (2,0,0), moving (0,1,0)
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(2, 0, Rotation2d.kZero), 0, 1, 0, 0, 0);
        // more r => lower limit
        assertEquals(-0.5, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(0.5, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

    @Test
    void testFar2() {
        TorqueConstraint jc = new TorqueConstraint(6);
        // at (3,0,0), moving (0,1,0)
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(3, 0, Rotation2d.kZero), 0, 1, 0, 0, 0);
        // more r => lower limit
        assertEquals(-0.333, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(0.333, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }

    @Test
    void testRealistic() {
        TorqueConstraint jc = new TorqueConstraint(30);
        // at (1,0,0), moving (0,1,0)
        Pose2dWithMotion state = new Pose2dWithMotion(
                new Pose2d(1, 0, Rotation2d.kZero), 0, 1, 0, 0, 0);
        // should match the constant constraint at around 1 m
        assertEquals(-5, jc.getMinMaxAcceleration(state, 0).getMinAccel(), DELTA);
        assertEquals(5, jc.getMinMaxAcceleration(state, 0).getMaxAccel(), DELTA);
    }
}
