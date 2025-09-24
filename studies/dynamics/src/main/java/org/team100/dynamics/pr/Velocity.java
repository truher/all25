package org.team100.dynamics.pr;

/**
 * Joint velocities for the PR example
 * 
 * @param q1dot velocity of the P joint
 * @param q2dot veloity of the R joint
 */
public record Velocity(double q1dot, double q2dot) {

}
