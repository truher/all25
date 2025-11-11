package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.team100.lib.geometry.GlobalVelocityR2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class InterceptTest {
    private static final double DELTA = 0.001;

    // @Test
    void testBothStationary() {
        // robot at the origin, stationary
        // target direction +x, stationary
        Optional<Rotation2d> azimuth = Intercept.intercept(
                new Translation2d(0, 0),
                new GlobalVelocityR2(0, 0),
                new Translation2d(1, 0),
                new GlobalVelocityR2(0, 0),
                1);
        assertTrue(azimuth.isPresent());
        assertEquals(0, azimuth.get().getRadians(), DELTA);
    }

    // @Test
    void testRobotStationaryTargetMoving() {
        // robot at the origin, stationary
        // target direction +x,-y, moving +y
        // projectile at 1 m/s, target at 1 m/s,
        // should intercept at y = 0
        Optional<Rotation2d> azimuth = Intercept.intercept(
                new Translation2d(0, 0),
                new GlobalVelocityR2(0, 0),
                new Translation2d(1, -1),
                new GlobalVelocityR2(0, 1),
                1);
        assertTrue(azimuth.isPresent());
        assertEquals(0, azimuth.get().getRadians(), DELTA);
    }

    // @Test
    void testRobotMovingTargetStationary() {
        // robot at the origin, moving +y
        // target direction +x,+y, stationary
        // projectile at 1 m/s in x, with robot at 1 m/s,
        // should intercept at y = 1
        Optional<Rotation2d> azimuth = Intercept.intercept(
                new Translation2d(0, 0),
                new GlobalVelocityR2(0, 1),
                new Translation2d(1, 1),
                new GlobalVelocityR2(0, 0),
                1);
        assertTrue(azimuth.isPresent());
        assertEquals(0, azimuth.get().getRadians(), DELTA);
    }

    // @Test
    void testTargetRecedingTooFast() {
        // robot at the origin, stationary
        // target direction +x, moving fast +x
        // projectile at 1 m/s in x, can't catch it.
        Optional<Rotation2d> azimuth = Intercept.intercept(
                new Translation2d(0, 0),
                new GlobalVelocityR2(0, 0),
                new Translation2d(1, 0),
                new GlobalVelocityR2(2, 0),
                1);
        assertTrue(azimuth.isEmpty());
    }

}