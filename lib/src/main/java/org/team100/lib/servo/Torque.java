package org.team100.lib.servo;

import java.util.function.DoubleUnaryOperator;

/** Represents extra torque for gravity and/or spring compensation. */
public class Torque {
    final DoubleUnaryOperator m_torque;

    public Torque(DoubleUnaryOperator torque) {
        m_torque = torque;
    }

    public double torque(double positionRad) {
        return m_torque.applyAsDouble(positionRad);
    }

}
