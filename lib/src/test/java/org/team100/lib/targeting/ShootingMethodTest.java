package org.team100.lib.targeting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.team100.lib.geometry.GlobalVelocityR2;

import edu.wpi.first.math.geometry.Translation2d;

public class ShootingMethodTest {

    @Test
    void testMotionless() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(range, 0.0001);
        Translation2d robotPosition = new Translation2d();
        GlobalVelocityR2 robotVelocity = GlobalVelocityR2.ZERO;
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        ShootingMethod.Solution x = o.orElseThrow();
        assertEquals(0, x.azimuth().getRadians(), 0.000001);
        assertEquals(0.156812, x.elevation().getRadians(), 0.000001);
        // check the range solution
        Range.Solution s = range.get(x.elevation().getRadians());
        assertEquals(1.999999, s.range(), 0.000001);
        assertEquals(0.2789, s.tof(), 0.0001);
    }

    @Test
    void testTowardsTarget() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(range, 0.0001);
        Translation2d robotPosition = new Translation2d();
        // driving towards the target
        GlobalVelocityR2 robotVelocity = new GlobalVelocityR2(1, 0);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        ShootingMethod.Solution x = o.orElseThrow();
        assertEquals(0, x.azimuth().getRadians(), 0.000001);
        // lower elevation
        assertEquals(0.129574, x.elevation().getRadians(), 0.000001);
        // check the range solution
        Range.Solution s = range.get(x.elevation().getRadians());
        // closer range
        assertEquals(1.764723, s.range(), 0.000001);
        // less time
        assertEquals(0.235276, s.tof(), 0.0001);
    }

    @Test
    void testAwayFromTarget() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(range, 0.0001);
        Translation2d robotPosition = new Translation2d();
        // driving away from the target
        GlobalVelocityR2 robotVelocity = new GlobalVelocityR2(-2, 0);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        ShootingMethod.Solution x = o.orElseThrow();
        assertEquals(0, x.azimuth().getRadians(), 0.000001);
        // higher elevation
        assertEquals(0.386036, x.elevation().getRadians(), 0.000001);
        // check the range solution
        Range.Solution s = range.get(x.elevation().getRadians());
        // longer range
        assertEquals(3.193693, s.range(), 0.000001);
        // more time
        assertEquals(0.596846, s.tof(), 0.0001);
    }

    @Test
    void testImpossible() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(range, 0.0001);
        Translation2d robotPosition = new Translation2d();
        // driving fast away from the target
        GlobalVelocityR2 robotVelocity = new GlobalVelocityR2(-10, 0);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        assertTrue(o.isEmpty());
    }

    @Test
    void testStrafing() {
        Drag d = new Drag(0.5, 0.025, 0.1, 0.1, 0.1);
        Range range = new Range(d, 10, 0);
        // tight tolerance for testing
        ShootingMethod m = new ShootingMethod(range, 0.0001);
        Translation2d robotPosition = new Translation2d();
        // driving to the left
        GlobalVelocityR2 robotVelocity = new GlobalVelocityR2(0, 2);
        // target is 2m away along +x
        Translation2d targetPosition = new Translation2d(2, 0);
        GlobalVelocityR2 targetVelocity = GlobalVelocityR2.ZERO;
        Optional<ShootingMethod.Solution> o = m.solve(
                robotPosition, robotVelocity, targetPosition, targetVelocity);
        ShootingMethod.Solution x = o.orElseThrow();
        // lag the target (to the right)
        assertEquals(-0.287561, x.azimuth().getRadians(), 0.000001);
        // a bit higher elevation
        assertEquals(0.167844, x.elevation().getRadians(), 0.000001);
        // check the range solution
        Range.Solution s = range.get(x.elevation().getRadians());
        // a bit longer range
        assertEquals(2.085642, s.range(), 0.000001);
        // a bit more time
        assertEquals(0.295766, s.tof(), 0.0001);
    }

}
