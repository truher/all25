package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;

public class TargetsTest {
    private static final double DELTA = 0.001;

    @Test
    void testTargets() {
        Pose2d p = new Pose2d(0, 0, Rotation2d.kZero);
        Targets t = new Targets((x) -> p, "Rotation3d");
        t.update();
        assertTrue(t.fieldRelativeTargets.isEmpty());
        // send some blips
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        inst.startClient4("tag_finder24");

        // test4 camera offset is 0,0,1, without rotation
        StructArrayTopic<Rotation3d> topic = inst.getStructArrayTopic(
                "objectVision/test4/5678/Rotation3d", Rotation3d.struct);
        StructArrayPublisher<Rotation3d> pub = topic.publish();
        // tilt down 45
        pub.set(new Rotation3d[] { new Rotation3d(0, Math.PI / 4, 0) });
        t.update();
        assertEquals(1, t.fieldRelativeTargets.size());
        Translation2d target = t.fieldRelativeTargets.get(0);
        // camera is 1m up, tilted 45 down, so target is 1m away
        assertEquals(1.0, target.getX(), DELTA);
        // target is on bore
        assertEquals(0, target.getY(), DELTA);

    }

    @Test
    void testTranslations() {

        SimulatedTargetWriter writer = new SimulatedTargetWriter();

        Pose2d p = new Pose2d(0, 0, Rotation2d.kZero);

        // need to instantiate the reader prior to the writer update because the poller
        // ignores things that came before.
        Targets reader = new Targets((x) -> p, "Rotation3d");

        Transform3d offset = Camera.get("test4").getOffset();
        writer.update(
                p, offset, new Translation2d[] {
                        new Translation2d(1, 0) });

        // getClosestTranslation does the update.
        // reader.update();

        Optional<Translation2d> tt = reader.getClosestTranslation2d();
        assertTrue(tt.isPresent());
        Translation2d ttt = tt.get();
        assertEquals(1.0, ttt.getX(), DELTA);
        assertEquals(0, ttt.getY(), DELTA);

        writer.close();
    }

}
