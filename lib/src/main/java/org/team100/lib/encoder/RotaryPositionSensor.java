package org.team100.lib.encoder;

import java.util.OptionalDouble;

/**
 * Absolute rotational measurement, as used in, for example, swerve steering,
 * arm angles, shooter angles, etc. This does not "wind up", it only returns
 * values within [-pi, pi]. This type of sensor cannot be "reset" at runtime,
 * offsets should be fixed at instantiation.
 * 
 * In 2025 these position sensors always sense the absolute position of a
 * mechanism, not through any sort of gearing.
 */
public interface RotaryPositionSensor  {

    /**
     * @return Counterclockwise-positive rad in [-pi,pi], empty if disconnected.
     */
    OptionalDouble getPositionRad();

    /**
     * Note some velocity implementations can be noisy.
     * 
     * @return Counterclockwise positive rad/s, empty if disconnected.
     */
    OptionalDouble getVelocityRad_S();

    /**
     * For logging.
     */
    void periodic();

    /**
     * Releases the encoder resource, if necessary (e.g. HAL ports).
     */
    void close();
}
