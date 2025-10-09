package org.team100.lib.encoder;

/** Represents motor-shaft encoder, probably some kind of built-in. */
public interface IncrementalBareEncoder {

    /**
     * Value should be updated in Robot.robotPeriodic().
     * 
     * @return rad
     */
    double getPositionRad();

    /**
     * Value should be updated in Robot.robotPeriodic().
     * 
     * Note some rate implementations can be noisy.
     * 
     * @return rad/s
     */
    double getVelocityRad_S();

    /**
     * Resets position to zero.
     *
     * Caches should also be flushed, so the new value is available immediately.
     */
    void reset();

    /**
     * Releases the encoder resource, if necessary (e.g. HAL ports).
     */
    void close();

    /**
     * Sets the incremental encoder position. This is only used to "zero" it, and
     * only done by the ProxyRotaryPositionSensor.
     * 
     * This is very slow, only use it on startup.
     * 
     * Caches should also be flushed, so the new value is available immediately.
     */
    void setEncoderPositionRad(double motorPositionRad);

    /** For logging */
    void periodic();

}
