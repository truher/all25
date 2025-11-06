package org.team100.frc2025;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.rev.CANSparkMotor;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerServo extends SubsystemBase {

    private final CANSparkMotor m_servo;
    private final DoubleLogger m_doubleLogger;

    public IndexerServo(LoggerFactory parent, int channel) {
        LoggerFactory logger = parent.type(this);
        m_doubleLogger = logger.doubleLogger(Level.TRACE, "Angle (deg)");
        m_servo = new Neo550CANSparkMotor(
            logger,
            new CanId(14),
            NeutralMode.BRAKE,
            MotorPhase.FORWARD,
            10,
            Feedforward100.makeNeo550(logger),
             PIDConstants.zero(logger));
    }

    public void set(double value) {
        m_servo.setDutyCycle(-1.0 * value);
        m_doubleLogger.log(() -> value);
    }

    public void stop() {
        m_servo.stop();
    }

    public Command feed() {
        return run(() -> set(1));
    }
}