package org.team100.lib.encoder;

/** Contains no logic. */
public class MockRotaryPositionSensor implements RotaryPositionSensor {
    public double angle = 0;
    public double rate = 0;

    @Override
    public double getPositionRad() {
        return angle;
    }

    @Override
    public double getVelocityRad_S() {
        return rate;
    }

    @Override
    public void periodic() {
    }

    @Override
    public void close() {
        //
    }

}
