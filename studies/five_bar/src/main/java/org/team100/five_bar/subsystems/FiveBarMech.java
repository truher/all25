package org.team100.five_bar.subsystems;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Control at the "mechanism" level, which in this case means sending position
 * commands to the motor controller, without feedforward.
 */
public class FiveBarMech extends SubsystemBase {
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 5;
    private static final double STATOR_LIMIT = 5;

    /** Left motor, "P1" in the diagram. */
    private final BareMotor m_motorP1;
    private final IncrementalBareEncoder m_encoderP1;
    /**
     * There's no absolute encoder in the apparatus, so we use a "proxy" instead;
     * this needs a "homing" mechanism of some kind.
     */
    private final ProxyRotaryPositionSensor m_sensorP1;
    private final RotaryMechanism m_mechP1;

    /** Right motor, "P5" in the diagram. */
    private final BareMotor m_motorP5;
    private final IncrementalBareEncoder m_encoderP5;
    private final ProxyRotaryPositionSensor m_sensorP5;
    private final RotaryMechanism m_mechP5;

    public FiveBarMech(LoggerFactory logger) {
        // zeros
        PIDConstants pid = new PIDConstants();
        Feedforward100 ff = Feedforward100.zero();

        LoggerFactory loggerP1 = logger.child("p1");
        Falcon6Motor motorP1 = new Falcon6Motor(
                loggerP1,
                1,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                pid,
                ff);
        m_motorP1 = motorP1;
        m_encoderP1 = new Talon6Encoder(loggerP1, motorP1);
        m_sensorP1 = new ProxyRotaryPositionSensor(m_encoderP1, 1.0);
        m_mechP1 = new RotaryMechanism(
                loggerP1,
                m_motorP1,
                m_sensorP1,
                1.0,
                0.0,
                1.0);

        LoggerFactory loggerP5 = logger.child("p5");
        Falcon6Motor motorP5 = new Falcon6Motor(
                loggerP5,
                2,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                pid,
                ff);
        m_motorP5 = motorP5;
        m_encoderP5 = new Talon6Encoder(loggerP5, motorP5);
        m_sensorP5 = new ProxyRotaryPositionSensor(m_encoderP5, 1.0);
        m_mechP5 = new RotaryMechanism(
                loggerP5,
                m_motorP5,
                m_sensorP5,
                1.0,
                0.0,
                1.0);
    }

    public void setPosition(double p1, double p5) {
        m_mechP1.setPosition(p1, 0, 0, 0);
        m_mechP5.setPosition(p5, 0, 0, 0);
    }

    //////////////////////

    private void setDutyCycle(double p1, double p5) {
        m_motorP1.setDutyCycle(p1);
        m_motorP5.setDutyCycle(p5);
    }

    private void resetEncoderPosition() {
        m_sensorP1.setEncoderPosition(0);
        m_sensorP5.setEncoderPosition(0);
    }

    ///////////////////////
    //
    // Commands

    public Command home() {
        return run(() -> setDutyCycle(0.05, 0.05));
    }

    public Command zero() {
        return run(this::resetEncoderPosition);
    }

    public Command position(DoubleSupplier p1, DoubleSupplier p5) {
        return run(() -> setPosition(p1.getAsDouble(), p5.getAsDouble()));
    }

}
