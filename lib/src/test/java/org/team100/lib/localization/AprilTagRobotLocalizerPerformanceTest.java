package org.team100.lib.localization;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;
import org.team100.lib.coherence.Takt;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.TestLoggerFactory;
import org.team100.lib.logging.primitive.TestPrimitiveLogger;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

class AprilTagRobotLocalizerPerformanceTest {
    private static final double DELTA = 0.01;
    private static final LoggerFactory logger = new TestLoggerFactory(new TestPrimitiveLogger());

    // uncomment this to run it. it consumes all the CPU.
    // @Test
    void testEstimateRobotPose2() throws IOException {
        // robot is panned right 45, translation is ignored.
        AprilTagFieldLayoutWithCorrectOrientation layout = new AprilTagFieldLayoutWithCorrectOrientation();
        final List<Pose2d> poseEstimate = new ArrayList<Pose2d>();
        final List<Double> timeEstimate = new ArrayList<Double>();
        PoseEstimator100 poseEstimator = new PoseEstimator100() {
            @Override
            public void put(double t, Pose2d p, double[] sd1, double[] sd2) {
                poseEstimate.add(p);
                timeEstimate.add(t);
            }

            @Override
            public SwerveModel get(double timestampSeconds) {
                return new SwerveModel(new Rotation2d(-Math.PI / 4));
            }
        };

        AprilTagRobotLocalizer vdp = new AprilTagRobotLocalizer(
                logger, layout, poseEstimator);

        // camera sees the tag straight ahead in the center of the frame,
        // but rotated pi/4 to the left. this is ignored anyway.
        Blip24 blip = new Blip24(7,
                new Transform3d(
                        new Translation3d(0, 0, Math.sqrt(2)),
                        new Rotation3d(0, -Math.PI / 4, 0)));

        // verify tag 7 location
        Pose3d tagPose = layout.getTagPose(Alliance.Red, 7).get();
        assertEquals(16.5791, tagPose.getX(), DELTA);
        assertEquals(2.663, tagPose.getY(), DELTA);
        assertEquals(1.451, tagPose.getZ(), DELTA);
        assertEquals(0, tagPose.getRotation().getX(), DELTA);
        assertEquals(0, tagPose.getRotation().getY(), DELTA);
        assertEquals(0, tagPose.getRotation().getZ(), DELTA);

        // default camera offset is no offset.
        final String cameraSerialNumber = "foo";
        final Blip24[] blips = new Blip24[] { blip };

        // run forever so i can use the profiler
        while (true)
            vdp.estimateRobotPose(cameraSerialNumber, blips, Takt.get(), Alliance.Red);
    }

    @Test
    void testNothing() {
        assertTrue(true);
    }

}
