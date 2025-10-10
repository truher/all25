package org.team100.lib.encoder;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.util.Util;

import edu.wpi.first.math.MathUtil;

/**
 * One of the kinds of absolute rotary position sensors directly connected to
 * the RoboRIO.
 */
public abstract class RoboRioRotaryPositionSensor implements RotaryPositionSensor {
    private static final double TWO_PI = 2.0 * Math.PI;

    private final double m_positionOffset;
    private final EncoderDrive m_drive;
    // LOGGERS
    private final DoubleLogger m_log_position;
    private final DoubleLogger m_log_position_turns;
    private final DoubleLogger m_log_position_turns_offset;

    protected RoboRioRotaryPositionSensor(
            LoggerFactory parent,
            double inputOffset,
            EncoderDrive drive) {
        LoggerFactory log = parent.type(this);
        m_positionOffset = Util.inRange(inputOffset, 0.0, 1.0);
        m_drive = drive;
        m_log_position = log.doubleLogger(Level.COMP, "position (rad)");
        m_log_position_turns = log.doubleLogger(Level.COMP, "position (turns)");
        m_log_position_turns_offset = log.doubleLogger(Level.TRACE, "position (turns-offset)");
    }

    /** Implementations should cache this. */
    protected abstract double getRatio();

    protected abstract double m_sensorMin();

    protected abstract double m_sensorMax();

    /** This should be nearly cached. */
    @Override
    public double getWrappedPositionRad() {
        double positionRad = getRad();
        m_log_position.log(() -> positionRad);
        return positionRad;
    }

    /** map to full [0,1] */
    protected double mapSensorRange(double pos) {
        // map sensor range
        if (pos < m_sensorMin()) {
            pos = m_sensorMin();
        }
        if (pos > m_sensorMax()) {
            pos = m_sensorMax();
        }
        pos = (pos - m_sensorMin()) / (m_sensorMax() - m_sensorMin());
        return pos;
    }

    /**
     * This should be nearly cached.
     * 
     * @return radians, [-pi, pi]
     */
    protected double getRad() {
        double ratio = getRatio();

        double posTurns = mapSensorRange(ratio);
        m_log_position_turns.log(() -> posTurns);

        double turnsMinusOffset = posTurns - m_positionOffset;
        m_log_position_turns_offset.log(() -> turnsMinusOffset);

        switch (m_drive) {
            case DIRECT:
                return MathUtil.angleModulus(turnsMinusOffset * TWO_PI);
            case INVERSE:
                return MathUtil.angleModulus(-1.0 * turnsMinusOffset * TWO_PI);
            default:
                throw new IllegalArgumentException();
        }
    }

    /**
     * Always returns zero.
     * 
     * Extracting velocity from the absolute position sensor does not work.
     */
    @Override
    public double getVelocityRad_S() {
        return 0;
    }

}
