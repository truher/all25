package org.team100.lib.dynamics.prr;

/**
 * Joint velocities for the PR example
 * 
 * @param q1dot velocity of the P joint
 * @param q2dot veloity of the first R joint
 * @param q3dot veloity of the second R joint
 */
public record PRRVelocity(double q1dot, double q2dot, double q3dot) {

}
