package org.team100.lib.testing;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class TestUtil {

    public static void verify(Map<String, Double> expected, Map<String, Double> actual) {
        assertEquals(expected.size(), actual.size());
        for (String key : expected.keySet()) {
            assertEquals(expected.get(key), actual.get(key), 1e-3, key);
        }
    }

    public static void verify(Pose3d expected, Map<String, Pose3d> poses, String name) {
        Pose3d actual = poses.get(name);
        verify(expected, actual, name);
    }

    public static void verify(Pose3d expected, Pose3d actual, String name) {
        assertEquals(expected.getX(), actual.getX(), 1e-3, name + " x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, name + " y");
        assertEquals(expected.getZ(), actual.getZ(), 1e-3, name + " z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), 1e-3, name + " rot x");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), 1e-3, name + " rot y");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), 1e-3, name + " rot z");
    }

    public static void verify(Transform3d expected, Transform3d actual) {
        assertEquals(expected.getX(), actual.getX(), 1e-3, " x");
        assertEquals(expected.getY(), actual.getY(), 1e-3, " y");
        assertEquals(expected.getZ(), actual.getZ(), 1e-3, " z");
        assertEquals(expected.getRotation().getX(), actual.getRotation().getX(), 1e-3, " rot x");
        assertEquals(expected.getRotation().getY(), actual.getRotation().getY(), 1e-3, " rot y");
        assertEquals(expected.getRotation().getZ(), actual.getRotation().getZ(), 1e-3, " rot z");
    }

    public static void print(Pose3d p) {
        System.out.printf("x %6.3f y %6.3f z %6.3f r %6.3f p %6.3f y %6.3f\n",
                p.getX(), p.getY(), p.getZ(),
                p.getRotation().getX(), p.getRotation().getY(), p.getRotation().getZ());
    }
}
