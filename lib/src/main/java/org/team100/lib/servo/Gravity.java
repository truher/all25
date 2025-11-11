package org.team100.lib.servo;

import java.util.function.DoubleUnaryOperator;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;

/**
 * Represents torque required to counteract gravity. This was used in 2025 for
 * the wrist joint.
 */
public class Gravity implements DoubleUnaryOperator {
    /** Max gravity torque, newton-meters */
    private final double m_gravityNm;
    /** Offset from horizontal */
    private final double m_offsetRad;
    private final DoubleLogger m_log_gravityTorque;
    private final DoubleLogger m_log_correctedPosition;

    /**
     * @param parent    log
     * @param gravityNm maximum torque
     * @param offsetRad position offset
     */
    public Gravity(LoggerFactory parent, double gravityNm, double offsetRad) {
        m_gravityNm = gravityNm;
        m_offsetRad = offsetRad;
        LoggerFactory log = parent.type(this);
        m_log_gravityTorque = log.doubleLogger(Level.TRACE, "gravity torque (Nm)");
        m_log_correctedPosition = log.doubleLogger(Level.TRACE, "corrected position (rad)");
    }

    @Override
    public double applyAsDouble(double mechanismPositionRad) {
        double correctedPosition = mechanismPositionRad + m_offsetRad;
        double gravityTorqueNm = m_gravityNm * -Math.sin(correctedPosition);
        m_log_correctedPosition.log(() -> correctedPosition);
        m_log_gravityTorque.log(() -> gravityTorqueNm);
        return gravityTorqueNm;
    }
}