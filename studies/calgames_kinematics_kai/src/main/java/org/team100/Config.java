package org.team100;

/**
 * @param shoulderHeight meters relative to the floor
 * @param shoulderAngle  radians relative to the base of the elevator, i.e. zero is
 *                       straight up
 * @param wristAngle     radians relative to the shoulder, i.e. zero is extending in the
 *                       same direction as the arm.
 */
public record Config(double shoulderHeight, double shoulderAngle, double wristAngle) {
};
