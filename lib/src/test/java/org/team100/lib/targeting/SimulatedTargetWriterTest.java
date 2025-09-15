package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;
import org.team100.lib.config.Camera;
import org.team100.lib.testing.Timeless;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

/** Timeless because the clock is used to decide to ignore (stale) input. */
public class SimulatedTargetWriterTest implements Timeless {
    private static final double DELTA = 0.001;

    @Test
    void testOne() {
        stepTime();
        SimulatedTargetWriter writer = new SimulatedTargetWriter();

        Pose2d p = new Pose2d(0, 0, Rotation2d.kZero);

        // need to instantiate the reader prior to the writer update because the poller
        // ignores things that came before.
        Targets reader = new Targets(
                (x) -> p,
                "objectVision",
                "Rotation3d");

        Transform3d offset = Camera.get("test4").getOffset();
        writer.update(
                p, offset, new Translation2d[] {
                        new Translation2d(1, 0) });

        reader.update();

        assertEquals(1, reader.getTargets().size());
        Translation2d target = reader.getTargets().get(0);
        // camera is 1m up, tilted 45 down, so target is 1m away
        assertEquals(1.0, target.getX(), DELTA);
        // target is on bore
        assertEquals(0, target.getY(), DELTA);

        writer.close();

    }

}
