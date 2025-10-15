package org.team100.lib.math;

import org.team100.lib.motion.drivetrain.state.GlobalSe2Velocity;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Velocity in R2, companion to Translation2d. */
public record GlobalR2Velocity(double x, double y) {
    public static GlobalR2Velocity fromPolar(Rotation2d angle, double speed) {
        return new GlobalR2Velocity(speed * angle.getCos(), speed * angle.getSin());
    }

    /** Pick up the translation component of v. */
    public static GlobalR2Velocity fromSe2(GlobalSe2Velocity v) {
        return new GlobalR2Velocity(v.x(), v.y());
    }

    public GlobalR2Velocity plus(GlobalR2Velocity other) {
        return new GlobalR2Velocity(x + other.x, y + other.y);
    }

    public Translation2d integrate(Translation2d start, double dt) {
        return new Translation2d(start.getX() + x * dt, start.getY() + y * dt);
    }
}
