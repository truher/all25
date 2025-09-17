package org.team100.lib.targeting;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.team100.lib.coherence.Takt;
import org.team100.lib.config.Camera;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
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
    private static final boolean DEBUG = false;

    private final Map<Camera, StructArrayPublisher<Rotation3d>> m_publishers;
    private final List<Camera> m_cameras;

    /** client instance, not the default */
    private final NetworkTableInstance m_inst;

    public SimulatedTargetWriter(
            List<Camera> cameras) {
        m_cameras = cameras;
        m_publishers = new HashMap<>();
        m_inst = NetworkTableInstance.getDefault();
        m_inst.startClient4("tag_finder24");
        m_inst.setServer("localhost");
        for (Camera camera : m_cameras) {
            String name = "objectVision/"
                    + camera.getSerial() + "/0/Rotation3d";
            m_publishers.put(
                    camera,
                    m_inst.getStructArrayTopic(
                            name, Rotation3d.struct).publish());
        }
    }

    public void update(
            Pose2d robotPose,
            Translation2d[] allTargets) {
        for (Map.Entry<Camera, StructArrayPublisher<Rotation3d>> entry : m_publishers.entrySet()) {
            Camera camera = entry.getKey();
            StructArrayPublisher<Rotation3d> publisher = entry.getValue();
            List<Rotation3d> rot = SimulatedObjectDetector.getRotations(
                    robotPose, camera.getOffset(), allTargets);
            if (DEBUG)
                Util.printf("rot size %d\n", rot.size());
            // tilt down 45
            // Rotation3d[] rots = new Rotation3d[] { new Rotation3d(0, Math.PI / 4, 0) };
            Rotation3d[] rots = rot.toArray(new Rotation3d[0]);
            double now = Takt.get();
            if (DEBUG)
                Util.printf("writer timestamp %f\n", now);
            publisher.set(rots, (long) (now * 1000000.0));
        }
    }

    public void close() {
        m_publishers.values().forEach(p -> p.close());
    }
}
