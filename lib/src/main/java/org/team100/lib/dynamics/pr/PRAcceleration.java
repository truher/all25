package org.team100.lib.dynamics.pr;

/**
 * Joint accelerations for the PR example
 * 
 * @param q1ddot acceleration of the P joint
 * @param q2ddot acceleration of the R joint
 */
public record PRAcceleration(double q1ddot, double q2ddot) {

}
