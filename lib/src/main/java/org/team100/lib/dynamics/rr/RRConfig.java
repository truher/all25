package org.team100.lib.dynamics.rr;

/**
 * Joint configuration for the RR example.
 * 
 * @param q1 rotation of joint 1 (CCW from x)
 * @param q2 rotation of joint 2 (CCW from link 1)
 */
public record RRConfig(double q1, double q2) {

}
