package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Twist2d;

public class UncertaintyTest {
    private static final double DELTA = 0.01;

    @Test
    void testVisionStdDevs() {
        double targetRangeM = 1.0;
        double[] visionStdDev = Uncertainty.visionMeasurementStdDevs(targetRangeM);
        assertEquals(3, visionStdDev.length);
        assertEquals(0.04, visionStdDev[0], DELTA);
        assertEquals(0.04, visionStdDev[1], DELTA);
        assertEquals(Double.MAX_VALUE, visionStdDev[2], DELTA);
    }

    @Test
    void testStateStdDevs() {
        // these are the "antijitter" values.
        // 1 mm, very low
        double[] stateStdDev = Uncertainty.TIGHT_STATE_STDDEV;
        assertEquals(3, stateStdDev.length);
        assertEquals(0.001, stateStdDev[0], DELTA);
        assertEquals(0.001, stateStdDev[1], DELTA);
        assertEquals(0.1, stateStdDev[2], DELTA);
    }

    @Test
    void testScaledTwist() {
        // 1 mm
        double[] stateStdDev = Uncertainty.TIGHT_STATE_STDDEV;
        double targetRangeM = 1.0;
        // 2 cm stdev, 20x
        double[] visionStdDev = Uncertainty.visionMeasurementStdDevs(targetRangeM);
        // 10 cm of difference between the vision update and the current pose
        Twist2d twist = new Twist2d(0.1, 0.1, 0);
        Twist2d scaled = Uncertainty.getScaledTwist(stateStdDev, visionStdDev, twist);
        // difference is discounted 20x
        assertEquals(0.002439, scaled.dx, 1e-6);
        assertEquals(0.002439, scaled.dy, 1e-6);
        assertEquals(0, scaled.dtheta, 1e-6);
    }

    @Test
    void testK() {
        double[] stateStdDev = Uncertainty.TIGHT_STATE_STDDEV;
        double targetRangeM = 1.0;
        double[] visionStdDev = Uncertainty.visionMeasurementStdDevs(targetRangeM);
        double[] k = Uncertainty.getK(stateStdDev, visionStdDev);
        assertEquals(3, k.length);
        assertEquals(0.024, k[0], DELTA);
        assertEquals(0.024, k[1], DELTA);
        assertEquals(0, k[2], DELTA);
    }

    @Test
    void testMix() {
        assertEquals(0.091, Uncertainty.mix(1, 100), DELTA);
        assertEquals(0.24, Uncertainty.mix(1, 10), DELTA);
        assertEquals(0.5, Uncertainty.mix(1, 1), DELTA);
        assertEquals(0.76, Uncertainty.mix(10, 1), DELTA);
        assertEquals(0.909, Uncertainty.mix(100, 1), DELTA);
    }

}
