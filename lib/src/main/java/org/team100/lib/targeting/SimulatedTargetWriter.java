package org.team100.lib.targeting;

import java.util.List;

import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

/**
 * Write simulated targets to Network Tables, so the Targets receiver can pick
 * them up.
 * 
 * TODO: simulate multiple cameras to see if the reader combines them
 */
public class SimulatedTargetWriter {
    private static final boolean DEBUG = true;

    // test4 camera offset is 0,0,1, without rotation
    private static final String CAMERA_IDENTITY = "test4";
    // the simulated pi has just one camera attached
    private static final String CAMERA_NUM = "0";
    StructArrayPublisher<Rotation3d> m_pub;

    public SimulatedTargetWriter() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("tag_finder24");
        m_pub = inst.getStructArrayTopic(
                "objectVision/"
                        + CAMERA_IDENTITY
                        + "/"
                        + CAMERA_NUM
                        + "/Rotation3d",
                Rotation3d.struct).publish();
    }

    public void update(
            Pose2d robotPose,
            Transform3d offset,
            Translation2d[] allTargets) {
        List<Rotation3d> rot = SimulatedObjectDetector.getRotations(
                robotPose, offset, allTargets);
        if (DEBUG)
            Util.printf("rot size %d\n", rot.size());
        // tilt down 45
        // Rotation3d[] rots = new Rotation3d[] { new Rotation3d(0, Math.PI / 4, 0) };
        Rotation3d[] rots = rot.toArray(new Rotation3d[0]);
        m_pub.set(rots);
    }

    public void close() {
        m_pub.close();
    }
}
