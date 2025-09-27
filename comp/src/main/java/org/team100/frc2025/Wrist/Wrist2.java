package org.team100.frc2025.Wrist;

import java.util.function.DoubleSupplier;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.Feedback100;
import org.team100.lib.controller.simple.FullStateFeedback;
import org.team100.lib.encoder.AS5048RotaryPositionSensor;
import org.team100.lib.encoder.EncoderDrive;
import org.team100.lib.encoder.RotaryPositionSensor;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.servo.AngularPositionServo;
import org.team100.lib.motion.servo.Gravity;
import org.team100.lib.motion.servo.OnboardAngularPositionServo;
import org.team100.lib.motion.servo.Spring;
import org.team100.lib.motion.servo.Torque;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;
import org.team100.lib.profile.timed.JerkLimitedIncrementalProfile;
import org.team100.lib.reference.ProfileReference1d;
import org.team100.lib.reference.Setpoints1d;
import org.team100.lib.reference.TimedProfileReference1d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The wrist class now handles gravity torque, it's not in a "servo".
 * 
 * TODO: caller should specify the acceleration of the carriage, since that
 * affects "gravity".
 */
public class Wrist2 extends SubsystemBase {

    private static final int GEAR_RATIO = 25;

    private static final double WRIST_MIN_POSITION = -0.5;
    private static final double WRIST_MAX_POSITION = 4;

    private static final double MAX_VEL = 40;
    private static final double MAX_ACCEL = 40;
    private static final double MAX_JERK = 40;

    // private LaserCan lc;

    private final AngularPositionServo m_wristServo;

    private final RotaryMechanism m_wristMech;
    private final Torque m_gravityAndSpringTorque;
    private final BooleanLogger m_log_safe;


    private boolean m_isSafe = false;

    /** carriage acceleration is reported by the elevator */
    public Wrist2(LoggerFactory parent, int wristID, DoubleSupplier carriageAccel) {
        LoggerFactory logger = parent.type(this);

        PIDConstants wristPID = PIDConstants.makeVelocityPID(0.3); // 0.3

        Feedforward100 wristFF = Feedforward100.makeKraken6Wrist();

        Feedback100 wristFeedback = new FullStateFeedback(
                logger, 4.5, 0.12, x -> x, 0.05, 0.05);

        JerkLimitedIncrementalProfile profile = new JerkLimitedIncrementalProfile(MAX_VEL, MAX_ACCEL, MAX_JERK, false);

        ProfileReference1d ref = new TimedProfileReference1d(profile);

        m_log_safe = logger.booleanLogger(Level.TRACE, "Wrist Safe Condition");

        switch (Identity.instance) {
            case COMP_BOT -> {
                int wristSupplyLimit = 60;
                int wristStatorLimit = 90;
                Kraken6Motor motor = new Kraken6Motor(logger, wristID, MotorPhase.REVERSE,
                        wristSupplyLimit, wristStatorLimit, wristPID, wristFF);

                // this reads the wrist angle directly.
                RotaryPositionSensor sensor = new AS5048RotaryPositionSensor(
                        logger,
                        5,
                        0.135541, // 0.346857, //0.317012, //0.227471, //0.188726
                        EncoderDrive.DIRECT);

                m_wristMech = new RotaryMechanism(
                        logger, motor, sensor,
                        GEAR_RATIO, WRIST_MIN_POSITION, WRIST_MAX_POSITION);

                m_wristServo = new OnboardAngularPositionServo(
                        logger, m_wristMech, ref, wristFeedback);

                double gravityNm = 9.0 + carriageAccel.getAsDouble();
                Gravity gravity = new Gravity(logger, gravityNm, -0.451230);
                Spring spring = new Spring(logger);
                m_gravityAndSpringTorque = new Torque((x) -> gravity.applyAsDouble(x) + spring.applyAsDouble(x));

            }
            default -> {

                SimulatedBareMotor motor = new SimulatedBareMotor(logger, 600);
                SimulatedBareEncoder encoder = new SimulatedBareEncoder(logger, motor);
                SimulatedRotaryPositionSensor sensor = new SimulatedRotaryPositionSensor(
                        logger, encoder, GEAR_RATIO);
                m_wristMech = new RotaryMechanism(
                        logger, motor, sensor,
                        GEAR_RATIO, WRIST_MIN_POSITION, WRIST_MAX_POSITION);

                m_wristServo = new OnboardAngularPositionServo(
                        logger, m_wristMech, ref, wristFeedback);

                // Gravity gravity = new Gravity(logger, 0, 0);
                // Spring spring = new Spring(logger);
                // m_gravityAndSpringTorque = new Torque((x) -> gravity.applyAsDouble(x) +
                // spring.applyAsDouble(x));
                // for now, no extra forces in simulation
                // TODO: add physics to the sim
                m_gravityAndSpringTorque = new Torque(x -> x);
            }

        }
        m_wristServo.reset();

    }

    @Override
    public void periodic() {
        m_wristServo.periodic();
        m_log_safe.log(() -> m_isSafe);

    }

    public void resetWristProfile() {
        m_wristServo.reset();
    }

    public boolean atSetpoint() {
        return m_wristServo.atSetpoint();
    }

    public boolean profileDone() {
        return m_wristServo.profileDone();
    }

    public boolean atGoal() {
        return m_wristServo.atGoal();
    }

    public void setWristDutyCycle(double value) {
        m_wristMech.setDutyCycle(value);
    }

    public double getAngle() {
        // note this default might be dangerous
        return m_wristServo.getPosition().orElse(0);
    }

    public void setAngle() {
        double torque = m_gravityAndSpringTorque.torque(m_wristServo.getPosition());
        m_wristServo.setPositionProfiled(0.2, torque);
    }

    /** set angle via profile */
    public void setAngleValue(double goal) {
        double torque = m_gravityAndSpringTorque.torque(m_wristServo.getPosition());
        m_wristServo.setPositionProfiled(goal, torque);
    }

    public void setAngleDirect(Setpoints1d setpoint) {
        double torque = m_gravityAndSpringTorque.torque(m_wristServo.getPosition());
        m_wristServo.setPositionDirect(setpoint, torque);
    }

    public void setAngleSafe() {
        double torque = m_gravityAndSpringTorque.torque(m_wristServo.getPosition());
        m_wristServo.setPositionProfiled(-0.1, torque);
    }

    public boolean getSafeCondition() {
        return m_isSafe;
    }

    public void setSafeCondition(boolean isSafe) {
        m_isSafe = isSafe;
    }

    public void stop() {
        m_wristServo.stop();
    }

    // OBSERVERS

    /** Don't drive if the wrist is too far out (dragging on the floor). */
    public boolean isSafeToDrive() {
        return getAngle() <= 0.9;
    }

    // COMMANDS

    /** Set the duty cycle perpetually. */
    public Command setDuty(double v) {
        return run(
                () -> setWristDutyCycle(v));
    }

    /** Use a profile to set the position perpetually. */
    public Command set(double v) {
        return runEnd(
                () -> setAngleValue(v),
                () -> setWristDutyCycle(0));
    }

    public Command set(DoubleSupplier v) {
        return run(
                () -> setAngleValue(v.getAsDouble()));
    }

    /** Move the wrist out of the way of the elevator. */
    public Command readyUp() {
        return set(0.4);
    }

}
