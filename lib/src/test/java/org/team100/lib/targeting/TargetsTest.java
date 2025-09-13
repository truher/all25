package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructArrayTopic;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class TargetsTest {
    private static final double DELTA = 0.001;

    @Test
    void testTargets() {
        Pose2d p = new Pose2d(0, 0, Rotation2d.kZero);
        Targets t = new Targets((x) -> p);
        t.update();
        assertTrue(t.objects.isEmpty());
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
        assertEquals(1, t.objects.size());
        Translation2d target = t.objects.get(0);
        // camera is 1m up, tilted 45 down, so target is 1m away
        assertEquals(1.0, target.getX(), DELTA);
        // target is on bore
        assertEquals(0, target.getY(), DELTA);

    }

    @Test
    void testTranslations() {
        // this uses the canned results that are presented for BLANK.
        // I think these are useful for simulation.
        Pose2d p = new Pose2d(1, 6, Rotation2d.kZero);
        Targets t = new Targets((x) -> p);
        Optional<Translation2d> tt = t.getClosestTranslation2d(Optional.of(Alliance.Blue));
        assertTrue(tt.isPresent());
        Translation2d ttt = tt.get();
        assertEquals(0.0, ttt.getX(), DELTA);
        assertEquals(0, ttt.getY(), DELTA);
    }

}
