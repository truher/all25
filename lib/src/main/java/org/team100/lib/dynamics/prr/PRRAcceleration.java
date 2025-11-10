package org.team100.lib.dynamics.prr;

/**
 * Joint accelerations for the PRR example
 * 
 * @param q1ddot acceleration of the P joint
 * @param q2ddot acceleration of the first R joint
 * @param q3ddot acceleration of the second R joint
 */
public record PRRAcceleration(double q1ddot, double q2ddot, double q3ddot) {

}
