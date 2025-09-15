package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;

public class VisionUpdaterTest {
    private static final double DELTA = 0.001;

    @Test
    void testScaledTwist() {
        // 1 mm
        double[] stateStdDev = AprilTagRobotLocalizer.tightStateStdDevs;
        double targetRangeM = 1.0;
        // 2 cm stdev, 20x
        double[] visionStdDev = AprilTagRobotLocalizer.visionMeasurementStdDevs(targetRangeM);
        // 10 cm of difference between the vision update and the current pose
        Twist2d twist = new Twist2d(0.1, 0.1, 0);
        Twist2d scaled = VisionUpdater.getScaledTwist(stateStdDev, visionStdDev, twist);
        // difference is discounted 20x
        assertEquals(0.002439, scaled.dx, 1e-6);
        assertEquals(0.002439, scaled.dy, 1e-6);
        assertEquals(0, scaled.dtheta, 1e-6);
    }

    /**
     * If the measurement is the same as the sample, nothing happens.
     */
    @Test
    void testZeroNudge() {
        double[] stateStdDev = new double[] {
                0.001,
                0.001,
                0.1 };
        double[] visionStdDev = new double[] {
                0.02,
                0.02,
                Double.MAX_VALUE };
        final Pose2d sample = new Pose2d();
        final Pose2d measurement = new Pose2d();
        final Pose2d nudged = VisionUpdater.nudge(sample, measurement, stateStdDev, visionStdDev);
        assertEquals(0, nudged.getX(), 1e-6);
        assertEquals(0, nudged.getY(), 1e-6);
        assertEquals(0, nudged.getRotation().getRadians(), 1e-6);

    }

    /**
     * Let's say the camera is correct, and it says the sample is 0.1 meters off.
     * How long does it take for the pose buffer to contain the right pose? Kind of
     * a long time.
     */
    @Test
    void testGentleNudge() {
        int frameRate = 50;
        double[] stateStdDev = new double[] {
                0.001,
                0.001,
                0.1 };
        double[] visionStdDev = new double[] {
                0.1,
                0.1,
                Double.MAX_VALUE };
        final Pose2d sample = new Pose2d();
        final Pose2d measurement = new Pose2d(0.1, 0, new Rotation2d());
        Pose2d nudged = sample;
        for (int i = 0; i < frameRate; ++i) {
            nudged = VisionUpdater.nudge(nudged, measurement, stateStdDev, visionStdDev);
        }
        // after 1 sec the error is about 6 cm which is too slow.
        Transform2d error = measurement.minus(nudged);
        assertEquals(0.061, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
        //
        for (int i = 0; i < frameRate; ++i) {
            nudged = VisionUpdater.nudge(nudged, measurement, stateStdDev, visionStdDev);
        }
        // after 2 sec the error is about 4 cm.
        error = measurement.minus(nudged);
        assertEquals(0.037, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
    }

    /**
     * Same as above with firmer updates. We want error under 2cm within about 1s.
     */
    @Test
    void testFirmerNudge() {
        int frameRate = 50;
        double[] stateStdDev = new double[] {
                0.001,
                0.001,
                0.1 };
        double[] visionStdDev = new double[] {
                0.03,
                0.03,
                Double.MAX_VALUE };
        final Pose2d sample = new Pose2d();
        final Pose2d measurement = new Pose2d(0.1, 0, new Rotation2d());
        Pose2d nudged = sample;
        for (int i = 0; i < frameRate; ++i) {
            nudged = VisionUpdater.nudge(nudged, measurement, stateStdDev, visionStdDev);
        }
        // after 1 sec the error is about 2 cm which is the target.
        Transform2d error = measurement.minus(nudged);
        assertEquals(0.019, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);
        //
        for (int i = 0; i < frameRate; ++i) {
            nudged = VisionUpdater.nudge(nudged, measurement, stateStdDev, visionStdDev);
        }
        // after 2 sec the error is about 4 mm which seems plenty tight
        error = measurement.minus(nudged);
        assertEquals(0.004, error.getX(), DELTA);
        assertEquals(0, error.getY(), DELTA);
        assertEquals(0, error.getRotation().getRadians(), DELTA);

    }

    @Test
    void testK() {
        double[] stateStdDev = AprilTagRobotLocalizer.tightStateStdDevs;
        double targetRangeM = 1.0;
        double[] visionStdDev = AprilTagRobotLocalizer.visionMeasurementStdDevs(targetRangeM);
        double[] k = VisionUpdater.getK(stateStdDev, visionStdDev);
        assertEquals(3, k.length);
        assertEquals(0.024, k[0], DELTA);
        assertEquals(0.024, k[1], DELTA);
        assertEquals(0, k[2], DELTA);
    }

    @Test
    void testMix() {
        assertEquals(0.091, VisionUpdater.mix(1, 100), DELTA);
        assertEquals(0.24, VisionUpdater.mix(1, 10), DELTA);
        assertEquals(0.5, VisionUpdater.mix(1, 1), DELTA);
        assertEquals(0.76, VisionUpdater.mix(10, 1), DELTA);
        assertEquals(0.909, VisionUpdater.mix(100, 1), DELTA);
    }

}
