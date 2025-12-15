package org.team100.lib.geometry;

/**
 * A direction (i.e. unit-length vector) in the SE3 manifold,
 * rigid transformations in three dimensions (which have six dimensions).
 * 
 * This is useful for representing spline controls for Pose3d.
 * 
 * It is exactly a unit-length Twist3d.
 */
public class DirectionSE3 {
    public final double x;
    public final double y;
    public final double z;
    public final double roll;
    public final double pitch;
    public final double yaw;

    public DirectionSE3(
            double px, double py, double pz,
            double proll, double ppitch, double pyaw) {
        double h = Math.sqrt(
                px * px + py * py + pz * pz
                        + proll * proll + ppitch * ppitch + pyaw * pyaw);
        x = px / h;
        y = py / h;
        z = pz / h;
        roll = proll / h;
        pitch = ppitch / h;
        yaw = pyaw / h;
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof DirectionSE3 other
                && Math.abs(other.x - x) < 1E-9
                && Math.abs(other.y - y) < 1E-9
                && Math.abs(other.z - z) < 1E-9
                && Math.abs(other.roll - roll) < 1E-9
                && Math.abs(other.pitch - pitch) < 1E-9
                && Math.abs(other.yaw - yaw) < 1E-9

        ;
    }

}
