package org.team100.dynamics.pr;

/**
 * Joint velocities for the PR example
 * 
 * @param q1ddot acceleration of the P joint
 * @param q2ddot acceleration of the R joint
 */
public record Acceleration(double q1ddot, double q2ddot) {

}
