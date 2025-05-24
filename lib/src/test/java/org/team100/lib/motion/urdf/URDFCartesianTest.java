package org.team100.lib.motion.urdf;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Map;

import org.junit.jupiter.api.Test;
import org.team100.lib.util.Util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N3;

public class URDFCartesianTest {
    private static final boolean DEBUG = false;

    @Test
    void testFwd() {
        URDFModel.Robot m = URDFCartesian.ROBOT;
        Map<String, Double> q = Map.of(
                "base_gantry", 0.0,
                "gantry_head", 0.0,
                "head_spindle", 0.0);
        Map<String, Pose3d> poses = m.forward(q);
        assertEquals(4, poses.size());
        verify(new Pose3d(0, 0, 0.0, new Rotation3d()), poses, "base_gantry");
        verify(new Pose3d(0, 0, 0.0, new Rotation3d()), poses, "gantry_head");
        verify(new Pose3d(0, 0, 0.0, new Rotation3d()), poses, "head_spindle");
        verify(new Pose3d(0, 0, -0.1, new Rotation3d()), poses, "center_point");
    }

    @Test
    void testInv() {
        URDFModel.Robot m = URDFCartesian.ROBOT;
        // desired end effector position is in the center of the
        // [0,1]x[0,1]x[0,0.1] workspace. Since the z extends -0.1
        // down, the actual z will be 0.15.
        Pose3d end = new Pose3d(0.5, 0.5, 0.05, new Rotation3d());
        // initial position is just somewhere random
        Vector<N3> q0 = VecBuilder.fill(0.1, 0.1, 0.1);
        Map<String, Double> qMap = m.inverse(
                Nat.N3(), q0, 1, "center_point", end);
        assertEquals(3, qMap.size());
        assertEquals(0.5, qMap.get("base_gantry"), 1e-3);
        assertEquals(0.5, qMap.get("gantry_head"), 1e-3);
        assertEquals(0.15, qMap.get("head_spindle"), 1e-3);
    }

    @Test
    void testInvBeyondLimit() {
        URDFModel.Robot m = URDFCartesian.ROBOT;
        // x position is beyond the [0,1] envelope, so it should return 1.
        Pose3d end = new Pose3d(1.5, 0.5, 0.05, new Rotation3d());
        // initial position is just somewhere random
        Vector<N3> q0 = VecBuilder.fill(0.1, 0.1, 0.1);
        Map<String, Double> qMap = m.inverse(
                Nat.N3(), q0, 1, "center_point", end);
        assertEquals(3, qMap.size());
        assertEquals(0.5, qMap.get("base_gantry"), 1e-3);
        // this is the coordinate that should be limited.
        assertEquals(1.0, qMap.get("gantry_head"), 1e-3);
        assertEquals(0.15, qMap.get("head_spindle"), 1e-3);
    }

    /** 52 us per solve on my laptop, 2 solver iterations each. */
    @Test
    void testInvPerformance() {
        URDFModel.Robot m = URDFCartesian.ROBOT;
        Pose3d end = new Pose3d(0.5, 0.5, 0.05, new Rotation3d());
        Vector<N3> q0 = VecBuilder.fill(0.1, 0.1, 0.1);
        int iterations = 10000;
        long startTime = System.currentTimeMillis();
        for (int i = 0; i < iterations; ++i) {
            m.inverse(Nat.N3(), q0, 1, "center_point", end);
        }
        long finishTime = System.currentTimeMillis();
        if (DEBUG) {
            Util.println("Cartesian inverse");
            Util.printf("ET (s): %6.3f\n", ((double) finishTime - startTime) / 1000);
            Util.printf("ET/call (ns): %6.3f\n ", 1000000 * ((double) finishTime - startTime) / iterations);
        }
    }

    void verify(Pose3d expected, Map<String, Pose3d> poses, String name) {
        Pose3d actual = poses.get(name);
        verify(expected, actual, name);
    }

    void verify(Pose3d expected, Pose3d actual, String name) {
        assertEquals(expected.getX(), actual.getX(), 1e-3, name + " x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, name + " y");
        assertEquals(expected.getZ(), actual.getZ(), 1e-3, name + " z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), 1e-3, name + " rot x");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), 1e-3, name + " rot y");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), 1e-3, name + " rot z");
    }
}
