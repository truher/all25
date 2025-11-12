package org.team100.frc2025;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.util.CanId;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerServo extends SubsystemBase {

    private final BareMotor m_servo;
    private final DoubleLogger m_logDutyCycle;

    public IndexerServo(LoggerFactory parent, int channel) {
        LoggerFactory logger = parent.type(this);
        m_logDutyCycle = logger.doubleLogger(Level.TRACE, "duty cycle");
        m_servo =  Neo550CANSparkMotor.get(
                logger,
                new CanId(14),
                MotorPhase.FORWARD,
                10,
                Feedforward100.makeNeo550(logger),
                PIDConstants.zero(logger));
    }

    public void set(double value) {
        m_servo.setDutyCycle(-1.0 * value);
        m_logDutyCycle.log(() -> value);
    }

    public double get() {
        return m_servo.getVelocityRad_S();
    }

    public void stop() {
        m_servo.stop();
    }

    public Command feed() {
        return run(() -> set(1));
    }
}