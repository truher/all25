package org.team100.lib.targeting;

import java.util.Optional;

import org.team100.lib.geometry.GlobalVelocityR2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * See INTERCEPT.md for details.
 */
public class Intercept {
    /**
     * Find a turret rotation which will intercept the target. If more than one
     * solution is possible, choose the sooner one. If no solution is possible,
     * return Optional.empty().
     * 
     * @param robotPosition  field-relative robot position, meters
     * @param robotVelocity  field-relative robot velocity, meters/sec
     * @param targetPosition field-relative target position, meters
     * @param targetVelocity field-relative target velocity, meters/sec
     * @param muzzleSpeed    speed of the projectile, meters/sec
     * @return field-relative firing solution azimuth
     */
    public static Optional<Rotation2d> intercept(
            Translation2d robotPosition,
            GlobalVelocityR2 robotVelocity,
            Translation2d targetPosition,
            GlobalVelocityR2 targetVelocity,
            double muzzleSpeed) {

        return Optional.empty();
    }

}
