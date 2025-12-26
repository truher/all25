package org.team100.lib.geometry;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class MetricsTest {
    private static final double DELTA = 0.001;

    @Test
    void testDistance() {
        // same pose => 0
        assertEquals(0,
                Metrics.projectedDistance(
                        new Pose2d(1, 0, Rotation2d.kZero),
                        new Pose2d(1, 0, Rotation2d.kZero)),
                DELTA);
        // 1d distance
        assertEquals(1,
                Metrics.projectedDistance(
                        new Pose2d(0, 0, Rotation2d.kZero),
                        new Pose2d(1, 0, Rotation2d.kZero)),
                DELTA);
        // 2d distance
        assertEquals(1.414,
                Metrics.projectedDistance(
                        new Pose2d(0, 1, Rotation2d.kZero),
                        new Pose2d(1, 0, Rotation2d.kZero)),
                DELTA);
        // rotation means a little arc, so the path length is a little longer.
        assertEquals(1.111,
                Metrics.projectedDistance(
                        new Pose2d(0, 0, Rotation2d.kZero),
                        new Pose2d(1, 0, Rotation2d.kCCW_Pi_2)),
                DELTA);
        // the arc in this case is the entire quarter circle
        assertEquals(1.571,
                Metrics.projectedDistance(
                        new Pose2d(0, 1, Rotation2d.kZero),
                        new Pose2d(1, 0, Rotation2d.kCCW_Pi_2)),
                DELTA);
        // order doesn't matter
        assertEquals(1.571,
                Metrics.projectedDistance(
                        new Pose2d(1, 0, Rotation2d.kCCW_Pi_2),
                        new Pose2d(0, 1, Rotation2d.kZero)),
                DELTA);
        // pure rotation yields zero distance, which isn't really what we want.
        assertEquals(0,
                Metrics.projectedDistance(
                        new Pose2d(0, 0, Rotation2d.kZero),
                        new Pose2d(0, 0, Rotation2d.kCCW_90deg)),
                DELTA);
        // use double geodesic distance to fix that.
        assertEquals(1.571,
                Metrics.doubleGeodesicDistance(
                        new Pose2d(0, 0, Rotation2d.kZero),
                        new Pose2d(0, 0, Rotation2d.kCCW_90deg)),
                DELTA);
        // pure rotation translation => zero
        assertEquals(0.0,
                Metrics.translationalDistance(
                        new Pose2d(0, 0, Rotation2d.kZero),
                        new Pose2d(0, 0, Rotation2d.kCCW_90deg)),
                DELTA);
    }

}
