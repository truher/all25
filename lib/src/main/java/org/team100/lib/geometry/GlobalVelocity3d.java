package org.team100.lib.geometry;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Velocity in three dimensions, companion to Translation3d.
 * 
 * This is different from VelocitySE2, which is the companion to Pose2d,
 * i.e. velocity of planar rigid transforms.
 */
public record GlobalVelocity3d(double x, double y, double z) {

    public static GlobalVelocity3d fromPolar(
            Rotation2d azimuth, Rotation2d elevation, double speed) {
        double vx = speed * azimuth.getCos() * elevation.getCos();
        double vy = speed * azimuth.getSin() * elevation.getCos();
        double vz = speed * elevation.getSin();
        return new GlobalVelocity3d(vx, vy, vz);
    }

    /** Pick up the translation component of v, in the XY plane. */
    public static GlobalVelocity3d fromSe2(VelocitySE2 v) {
        return new GlobalVelocity3d(v.x(), v.y(), 0);
    }

    public GlobalVelocity3d plus(GlobalVelocity3d other) {
        return new GlobalVelocity3d(x + other.x, y + other.y, z + other.z);
    }

    public double normXY() {
        return Math.sqrt(x * x + y * y);
    }

}
