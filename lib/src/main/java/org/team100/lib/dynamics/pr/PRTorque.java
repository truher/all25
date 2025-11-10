package org.team100.lib.dynamics.pr;

/**
 * Force or torque for the PR example.
 * 
 * @param f1 force on the P joint
 * @param t2 torque on the R joint
 */
public record PRTorque(double f1, double t2) {

}
