package org.team100.lib.sensor.position.absolute;

/**
 * Describes how the encoder angle is linked to the steering angle.
 * 
 * Note: currently we use this to correct the (inverted) AS5048 output, which is
 * not a good use of this concept. We should fix the AS5048 class so it produces
 * conventional output.
 */
public enum EncoderDrive {
    /**
     * Encoder moves the same as the module, e.g. via a belt, as in the WCP modules,
     * or via direct drive, as in the SDS modules, or via two gears in the AM
     * modules.
     */
    DIRECT,
    /**
     * Encoder moves opposite to the module, e.g. via a single gear, as in the
     * Team100 adaptation of the AM modules.
     */
    INVERSE
}