package org.team100.lib.dynamics.prr;

/**
 * Force or torque for the PR example.
 * 
 * @param f1 force on the P joint
 * @param t2 torque on the first R joint
 * @param t3 torque on the second R joint
 */
public record PRRTorque(double f1, double t2, double t3) {

}
