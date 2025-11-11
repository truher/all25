package org.team100.lib.dynamics.prr;

/**
 * Joint configuration for the PR example.
 * 
 * @param q1 extension of the P joint (+x)
 * @param q2 rotation of the first R joint (CCW from +x)
 * @param q3 rotation of the second R joint (CCW from the first)
 */
public record PRRConfig(double q1, double q2, double q3) {

}
