package org.team100.lib.sensor.switch100.rev;

import java.util.function.BooleanSupplier;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.motor.rev.CANSparkMotor;

/** Represents one of the "limit switch" inputs of a REV CANSpark controller */
public class CANSparkLimitSwitch {
    public enum Direction {
        FORWARD, REVERSE
    }

    private final BooleanLogger m_log;
    private final BooleanSupplier m_switch;

    public CANSparkLimitSwitch(LoggerFactory log, CANSparkMotor motor, Direction dir) {
        m_log = log.type(this).booleanLogger(Level.TRACE, "state");
        m_switch = switch (dir) {
            case FORWARD -> motor::getForwardLimitSwitch;
            case REVERSE -> motor::getReverseLimitSwitch;
        };
    }

    public boolean get() {
        boolean state = m_switch.getAsBoolean();
        m_log.log(() -> state);
        return state;
    }

}
