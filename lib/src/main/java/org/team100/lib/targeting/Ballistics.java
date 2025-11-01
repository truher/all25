package org.team100.lib.targeting;

/**
 * Given the shooter muzzle speed and elevation, predict landing range and time
 * of flight.
 */
public class Ballistics {
    /** gravity, m/s/s */
    private static final double G = 9.81;

    /**
     * @param range in meters
     * @param tof   time of flight, seconds
     */
    public record Solution(double range, double tof) {
    }

    /**
     * The parabolic path is certainly wrong; this is only for testing.
     * 
     * @param speed     muzzle speed, meters/sec
     * @param elevation shooter elevation above horizontal, radians
     * @return landing solution
     */
    public static Solution parabolic(double speed, double elevation) {
        // https://en.wikipedia.org/wiki/Projectile_motion#Time_of_flight_or_total_time_of_the_whole_journey
        double tof = 2 * speed * Math.sin(elevation) / G;
        double range = speed * tof * Math.cos(elevation);
        return new Solution(range, tof);
    }

    /**
     * Computes the elevation required for the given range. This is useful to find
     * the starting point for optimization.
     * 
     * @param speed muzzle speed, meters/sec
     * @param range distance to target, meters
     * @return elevation required, radians
     */
    public static double parabolicElevation(double speed, double range) {
        return 0;
    }

    /**
     * Path with Newton drag (proportional to the square of velocity).
     * 
     * https://en.wikipedia.org/wiki/Projectile_motion#Trajectory_of_a_projectile_with_Newton_drag
     * 
     * @param speed     muzzle speed, meters/sec
     * @param elevation shooter elevation above horizontal, radians
     * @return landing solution
     */
    public static Solution newton(double speed, double elevation) {
        // this should use a precomputed lookup table.
        return new Solution(0, 0);
    }

}
