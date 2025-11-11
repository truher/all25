package org.team100.lib.subsystems.lynxmotion_arm;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;

public class AnalyticTwoDofKinematicsTest {
    private static final double DELTA = 0.001;

    @Test
    void testf1() {
        // stretched along x
        AnalyticTwoDofKinematics k = new AnalyticTwoDofKinematics(1, 1);
        TwoDofArmPosition p = new TwoDofArmPosition(
                new Translation2d(1, 0), new Translation2d(2, 0));
        TwoDofArmConfig q = new TwoDofArmConfig(0, 0);
        verify(k, p, q);
    }

    @Test
    void testf2() {
        // up and then out
        AnalyticTwoDofKinematics k = new AnalyticTwoDofKinematics(1, 1);
        TwoDofArmPosition p = new TwoDofArmPosition(
                new Translation2d(0, 1), new Translation2d(1, 1));
        TwoDofArmConfig q = new TwoDofArmConfig(Math.PI / 2, -1 * Math.PI / 2);
        verify(k, p, q);
    }

    @Test
    void testf3() {
        // equilateral triangle, first link up
        AnalyticTwoDofKinematics k = new AnalyticTwoDofKinematics(1, 1);
        TwoDofArmPosition p = new TwoDofArmPosition(
                new Translation2d(0, 1), new Translation2d(Math.sqrt(3) / 2, 0.5));
        TwoDofArmConfig q = new TwoDofArmConfig(Math.PI / 2, -2 * Math.PI / 3);
        verify(k, p, q);
    }

    @Test
    void test4() {
        // vertical equilateral triangle
        AnalyticTwoDofKinematics k = new AnalyticTwoDofKinematics(1, 1);
        TwoDofArmPosition p = new TwoDofArmPosition(
                new Translation2d(-Math.sqrt(3) / 2, 0.5), new Translation2d(0, 1));
        TwoDofArmConfig q = new TwoDofArmConfig(5 * Math.PI / 6, -2 * Math.PI / 3);
        verify(k, p, q);
    }

    @Test
    void test5() {
        // behind
        AnalyticTwoDofKinematics k = new AnalyticTwoDofKinematics(1, 1);
        TwoDofArmPosition p = new TwoDofArmPosition(
                new Translation2d(-1, 0), new Translation2d(-1, 1));
        TwoDofArmConfig q = new TwoDofArmConfig(Math.PI, -Math.PI / 2);
        verify(k, p, q);
    }

    void verify(AnalyticTwoDofKinematics k, TwoDofArmPosition p, TwoDofArmConfig q) {
        verifyFwd(p, k.forward(q));
        verifyInv(q, k.inverse(p.p2()));
    }

    void verifyFwd(TwoDofArmPosition expected, TwoDofArmPosition actual) {
        assertEquals(expected.p1(), actual.p1(), "fwd p1");
        assertEquals(expected.p2(), actual.p2(), "fwd p2");
    }

    void verifyInv(TwoDofArmConfig expected, TwoDofArmConfig actual) {
        assertEquals(expected.q1(), actual.q1(), DELTA, "inv q1");
        assertEquals(expected.q2(), actual.q2(), DELTA, "inv q2");
    }
}