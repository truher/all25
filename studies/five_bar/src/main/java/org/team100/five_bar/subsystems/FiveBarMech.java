package org.team100.five_bar.subsystems;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.ProxyRotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.five_bar.FiveBarKinematics;
import org.team100.lib.motion.five_bar.JointPositions;
import org.team100.lib.motion.five_bar.Scenario;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motor.Falcon6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Control at the "mechanism" level, which in this case means sending position
 * commands to the motor controller, without feedforward.
 */
public class FiveBarMech extends SubsystemBase {
    private static final boolean DEBUG = false;
    /** Low current limits */
    private static final double SUPPLY_LIMIT = 5;
    private static final double STATOR_LIMIT = 5;
    /** Units of positional PID are volts per revolution. */
    private static final PIDConstants PID = PIDConstants.makePositionPID(10.0);
    /** We never use feedforward since all our goals are motionless. */
    private static final Feedforward100 FF = Feedforward100.zero();
    private static final Scenario SCENARIO;
    static {
        // origin is P1
        SCENARIO = new Scenario();
        // TODO: real measurements
        SCENARIO.a1 = 0.1;
        SCENARIO.a2 = 0.1;
        SCENARIO.a3 = 0.1;
        SCENARIO.a4 = 0.1;
        SCENARIO.a5 = 0.1;
        SCENARIO.xcenter = 0.5;
        SCENARIO.ycenter = 0.15;
    }

    /** Left motor, "P1" in the diagram. */
    private final RotaryMechanism m_mechP1;
    /** Right motor, "P5" in the diagram. */
    private final RotaryMechanism m_mechP5;

    /**
     * There's no absolute encoder in the apparatus, so we use a "proxy" instead;
     * this needs a "homing" mechanism of some kind.
     */
    private final ProxyRotaryPositionSensor m_sensorP1;
    private final ProxyRotaryPositionSensor m_sensorP5;

    public FiveBarMech(LoggerFactory logger) {
        LoggerFactory loggerP1 = logger.child("p1");
        LoggerFactory loggerP5 = logger.child("p5");
        switch (Identity.instance) {
            case COMP_BOT -> {
                Falcon6Motor motorP1 = makeMotor(loggerP1, 1);
                Falcon6Motor motorP5 = makeMotor(loggerP5, 2);

                m_sensorP1 = new ProxyRotaryPositionSensor(
                        new Talon6Encoder(loggerP1, motorP1), 1.0);
                m_sensorP5 = new ProxyRotaryPositionSensor(
                        new Talon6Encoder(loggerP5, motorP5), 1.0);

                m_mechP1 = new RotaryMechanism(
                        loggerP1,
                        motorP1,
                        m_sensorP1,
                        1.0,
                        0.0,
                        1.0);
                m_mechP5 = new RotaryMechanism(
                        loggerP5,
                        motorP5,
                        m_sensorP5,
                        1.0,
                        0.0,
                        1.0);
            }
            default -> {
                SimulatedBareMotor motorP1 = new SimulatedBareMotor(loggerP1, 600);
                SimulatedBareMotor motorP5 = new SimulatedBareMotor(loggerP5, 600);

                m_sensorP1 = new ProxyRotaryPositionSensor(
                        new SimulatedBareEncoder(logger, motorP1), 1.0);
                m_sensorP5 = new ProxyRotaryPositionSensor(
                        new SimulatedBareEncoder(logger, motorP5), 1.0);

                m_mechP1 = new RotaryMechanism(
                        loggerP1,
                        motorP1,
                        m_sensorP1,
                        1.0,
                        -100.0,
                        100.0);
                m_mechP5 = new RotaryMechanism(
                        loggerP5,
                        motorP5,
                        m_sensorP5,
                        1.0,
                        -100.0,
                        100.0);
            }
        }
    }

    /** Update position by adding. */
    public void add(double p1, double p5) {
        m_mechP1.setPosition(m_mechP1.getPositionRad().orElseThrow() + p1, 0, 0, 0);
        m_mechP5.setPosition(m_mechP5.getPositionRad().orElseThrow() + p5, 0, 0, 0);
    }

    /** Set position goal, motionless. */
    public void setPosition(double p1, double p5) {
        if (DEBUG)
            Util.printf("FiveBarMech.setPosition %f %f\n", p1, p5);
        m_mechP1.setPosition(p1, 0, 0, 0);
        m_mechP5.setPosition(p5, 0, 0, 0);
    }

    public JointPositions getJointPositions() {
        double q1 = m_mechP1.getPositionRad().orElseThrow();
        double q5 = m_mechP5.getPositionRad().orElseThrow();
        if (DEBUG)
            Util.printf("joint positions %f %f\n", q1, q5);
        return FiveBarKinematics.forward(SCENARIO, q1, q5);
    }

    @Override
    public void periodic() {
        m_mechP1.periodic();
        m_mechP5.periodic();
    }

    //////////////////////

    private Falcon6Motor makeMotor(LoggerFactory logger, int canId) {
        return new Falcon6Motor(
                logger,
                canId,
                MotorPhase.FORWARD,
                SUPPLY_LIMIT,
                STATOR_LIMIT,
                PID,
                FF);
    }

    /** For homing. */
    private void setDutyCycle(double p1, double p5) {
        m_mechP1.setDutyCycleUnlimited(p1);
        m_mechP5.setDutyCycleUnlimited(p5);
    }

    /** For zeroing. */
    private void resetEncoderPosition() {
        m_sensorP1.setEncoderPosition(0);
        m_sensorP5.setEncoderPosition(0);
    }

    ///////////////////////
    //
    // Commands

    public Command home() {
        return run(() -> setDutyCycle(0.01, 0.01));
    }

    public Command zero() {
        return runOnce(this::resetEncoderPosition);
    }

    /** Update position by adding. */
    public Command position(DoubleSupplier p1, DoubleSupplier p5) {
        return run(() -> add(p1.getAsDouble(), p5.getAsDouble()));
    }

}
