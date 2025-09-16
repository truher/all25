package org.team100.lib.encoder;

import java.util.OptionalDouble;

/**
 * An encoder implementation that doesn't do anything.
 * This is useful if you want to make a mechanism (so you can have gears) but
 * you are using a primitive motor controller and no separate encoder anywhere.
 */
public class NoEncoder implements IncrementalBareEncoder {

    @Override
    public OptionalDouble getVelocityRad_S() {
        return OptionalDouble.empty();
    }

    @Override
    public OptionalDouble getPositionRad() {
        return OptionalDouble.empty();
    }

    @Override
    public void reset() {
    }

    @Override
    public void close() {
    }

    @Override
    public void setEncoderPositionRad(double motorPositionRad) {
    }

    @Override
    public void periodic() {
    }

}
