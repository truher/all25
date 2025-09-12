package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;

public class TargetsTest {

    @Test
    void testTargets() {
        Pose2d p = new Pose2d(2.6576, 4.0259, Rotation2d.kZero);
        Targets t = new Targets((x) -> p);
        t.update();
        assertTrue(t.objects.isEmpty());
        // send some blips
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("tag_finder24");

        StructArrayTopic<Rotation3d> topic = inst.getStructArrayTopic(
                "objectVision/1234/5678/Rotation3d", Rotation3d.struct);
        StructArrayPublisher<Rotation3d> pub = topic.publish();
        pub.set(new Rotation3d[] { new Rotation3d(0, 0, 0) });
        t.update();
        // skip first update
        assertTrue(t.objects.isEmpty());
        // jitter so NT passes it
        pub.set(new Rotation3d[] { new Rotation3d(0.01, 0, 0) });
        t.update();
        // TODO: finish this test
        // assertFalse(t.objects.isEmpty());

    }

}
