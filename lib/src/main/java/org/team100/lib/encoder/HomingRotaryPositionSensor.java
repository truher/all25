package org.team100.lib.encoder;

import java.util.OptionalDouble;

import edu.wpi.first.math.MathUtil;

/**
 * Proxies a RotaryPositionSensor with an offset.
 * 
 * This is intended for "homing" scenarios, where there is no actual absolute
 * sensor, but instead a mechanism hits a hard stop (or triggers a switch or
 * whatever) at a defined location.
 */
public class HomingRotaryPositionSensor implements RotaryPositionSensor {
    private final RotaryPositionSensor m_sensor;
    private double m_offset;

    public HomingRotaryPositionSensor(RotaryPositionSensor sensor) {
        m_sensor = sensor;
        m_offset = 0;
    }

    /**
     * Adjust the offset so that the returned position is x.
     * If the delegate returns empty, do nothing.
     * You should call this at the "homing position".
     */
    public void setPosition(double x) {
        OptionalDouble val = m_sensor.getPositionRad();
        if (val.isEmpty())
            return;
        double p = val.getAsDouble();
        m_offset = x - p;
    }

    @Override
    public OptionalDouble getPositionRad() {
        OptionalDouble val = m_sensor.getPositionRad();
        if (val.isEmpty())
            return val;
        double p = val.getAsDouble();
        return OptionalDouble.of(MathUtil.angleModulus(p + m_offset));
    }

    /** Velocity is independent of offset. */
    @Override
    public OptionalDouble getVelocityRad_S() {
        return m_sensor.getVelocityRad_S();
    }

    @Override
    public void periodic() {
        m_sensor.periodic();
    }

    @Override
    public void close() {
        m_sensor.close();
    }
}
