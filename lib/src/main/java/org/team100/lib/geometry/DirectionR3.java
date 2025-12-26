package org.team100.lib.geometry;

/**
 * Represents a direction in 3d space as a unit vector in R3.
 * 
 * This is a different idea than Rotation3d, which specifies an angular
 * *transformation* in 3d space.
 * 
 * Possible parameterizations:
 * 
 * * theta (equatorial), phi (polar)
 * * elevation (above horizontal), azimuth (equatorial)
 * * direction cosines (3 parameters with L2 norm = 1)
 * 
 * First let's try direction cosines.
 * 
 * l = cos(alpha)
 * m = cos(beta)
 * n = cos(gamma)
 * 
 * @see https://en.wikipedia.org/wiki/Direction_cosine
 */
public class DirectionR3 {
    public final double x;
    public final double y;
    public final double z;

    public DirectionR3(double px, double py, double pz) {
        double h = Math.sqrt(px * px + py * py + pz * pz);
        x = px / h;
        y = py / h;
        z = pz / h;
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof DirectionR3 other
                && Math.abs(other.x - x) < 1E-9
                && Math.abs(other.y - y) < 1E-9
                && Math.abs(other.z - z) < 1E-9;
    }

}
