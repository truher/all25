package frc.robot;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;
import org.team100.lib.reference.IncrementalProfileReference1d;
import org.team100.lib.reference.ProfileReference1d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Implements several different control methods.
 */
public class FiveBar extends SubsystemBase {
    private static final double POSITION_TOLERANCE = 0.05;
    private static final double VELOCITY_TOLERANCE = 0.05;
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 5;
    private static final double STATOR_LIMIT = 5;
    private static final double maxVel = 190;
    private static final double maxAccel = 210;
    private static final double kPositionTolerance = 0.01;

    /** Left motor, "P1" in the diagram. */
    private final BareMotor m_motorP1;
    private final IncrementalBareEncoder m_encoderP1;
    /**
     * There's no absolute encoder in the apparatus, so we use a "proxy" instead;
     * this needs a "homing" mechanism of some kind.
     */
    private final ProxyRotaryPositionSensor m_sensorP1;
    private final RotaryMechanism m_mechP1;
    private final OutboardAngularPositionServo m_servoP1;

    /** Right motor, "P5" in the diagram. */
    private final BareMotor m_motorP5;
    private final IncrementalBareEncoder m_encoderP5;
    private final ProxyRotaryPositionSensor m_sensorP5;
    private final RotaryMechanism m_mechP5;
    private final OutboardAngularPositionServo m_servoP5;

    public FiveBar(LoggerFactory logger) {
        // zeros
        PIDConstants pid = new PIDConstants();
        Feedforward100 ff = Feedforward100.zero();
        Profile100 profile = new TrapezoidProfile100(
                maxVel, maxAccel, kPositionTolerance);

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

        ProfileReference1d refP1 = new IncrementalProfileReference1d(
                profile, POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        m_servoP1 = new OutboardAngularPositionServo(
                loggerP1,
                m_mechP1,
                refP1);

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
        ProfileReference1d refP5 = new IncrementalProfileReference1d(
                profile, POSITION_TOLERANCE, VELOCITY_TOLERANCE);
        m_servoP5 = new OutboardAngularPositionServo(
                loggerP5,
                m_mechP5,
                refP5);
    }

    public void setDutyCycle(double p1, double p5) {
        m_motorP1.setDutyCycle(p1);
        m_motorP5.setDutyCycle(p5);
    }

    public void resetEncoderPosition() {
        m_sensorP1.setEncoderPosition(0);
        m_sensorP5.setEncoderPosition(0);
    }

    public Command home() {
        return run(() -> setDutyCycle(0.05, 0.05));
    }

    public Command zero() {
        return run(this::resetEncoderPosition);
    }

    public Command dutyCycle(DoubleSupplier p1, DoubleSupplier p5) {
        return run(() -> setDutyCycle(p1.getAsDouble(), p5.getAsDouble()));
    }
}
