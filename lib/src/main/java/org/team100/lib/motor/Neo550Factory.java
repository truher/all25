package org.team100.lib.motor;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.controller.simple.IncrementalProfiledController;
import org.team100.lib.controller.simple.ProfiledController;
import org.team100.lib.controller.simple.ZeroFeedback;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.encoder.CombinedEncoder;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.SimulatedRotaryPositionSensor;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LimitedRotaryMechanism;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.RotaryMechanism;
import org.team100.lib.motion.mechanism.SimpleLinearMechanism;
import org.team100.lib.motion.mechanism.SimpleRotaryMechanism;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardAngularPositionServo;
import org.team100.lib.motion.servo.OutboardGravityServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.profile.incremental.Profile100;
import org.team100.lib.profile.incremental.TrapezoidProfile100;

public class Neo550Factory {

    public static LinearMechanism getNEO550LinearMechanism(
            String name,
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        LoggerFactory moduleLogger = parent.child(name);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                moduleLogger,
                canID,
                motorPhase,
                currentLimit,
                Feedforward100.makeNeo550(),
                new PIDConstants());
        return new SimpleLinearMechanism(
                motor,
                new CANSparkEncoder(moduleLogger, motor),
                gearRatio,
                wheelDiameterM);
    }

    public static RotaryMechanism getNEO550RotaryMechanism(
            String name,
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase) {
        LoggerFactory moduleLogger = parent.child(name);
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                moduleLogger,
                canID,
                motorPhase,
                currentLimit,
                Feedforward100.makeNeo550(),
                PIDConstants.makePositionPID(1));
        return new SimpleRotaryMechanism(
                moduleLogger,
                motor,
                new CANSparkEncoder(moduleLogger, motor),
                gearRatio);
    }

    /**
     * Uses simulated position sensors, must be used with clock control (e.g.
     * {@link Timeless}).
     */
    public static OutboardGravityServo getNEO550GravityServo(
            String name,
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double p,
            double gravityNm,
            double offsetRad,
            double lowerLimit,
            double upperLimit) {
        LoggerFactory moduleLogger = parent.child(name);
        Neo550CANSparkMotor driveMotor = new Neo550CANSparkMotor(moduleLogger, canID, motorPhase, currentLimit,
                Feedforward100.makeNeo550(), PIDConstants.makePositionPID(p));
        RotaryMechanism rotaryMechanism = new LimitedRotaryMechanism(new SimpleRotaryMechanism(moduleLogger, driveMotor,
                new CANSparkEncoder(moduleLogger, driveMotor), gearRatio), lowerLimit, upperLimit);
        Profile100 profile = new TrapezoidProfile100(1, 1, 0.01);
        ZeroFeedback feedback = new ZeroFeedback(x -> x, 0.01, 0.01);
        ProfiledController controller = new IncrementalProfiledController(
                moduleLogger, profile, feedback, x -> x, 0.05, 0.05);
        return new OutboardGravityServo(
                parent,
                new OutboardAngularPositionServo(
                        moduleLogger,
                        rotaryMechanism,
                        new CombinedEncoder(
                                moduleLogger,
                                new SimulatedRotaryPositionSensor(moduleLogger, rotaryMechanism, () -> 0),
                                rotaryMechanism),
                        // true),
                        controller),
                gravityNm,
                offsetRad);
    }

    public static LinearVelocityServo getNEO550VelocityServo(
            String name,
            LoggerFactory parent,
            int currentLimit,
            int canID,
            double gearRatio,
            MotorPhase motorPhase,
            double wheelDiameterM) {
        LoggerFactory moduleLogger = parent.child(name);
        return new OutboardLinearVelocityServo(moduleLogger, getNEO550LinearMechanism(name,
                moduleLogger,
                currentLimit,
                canID,
                gearRatio,
                motorPhase,
                wheelDiameterM));
    }

    public static OutboardLinearVelocityServo simulatedDriveServo(LoggerFactory parent) {
        return new OutboardLinearVelocityServo(
                parent,
                simulatedLinearMechanism(parent));
    }

    /**
     * Uses simulated position sensors, must be used with clock control (e.g.
     * {@link Timeless}).
     */
    public static OutboardGravityServo simulatedGravityServo(LoggerFactory parent) {
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        RotaryMechanism rotaryMechanism = new SimpleRotaryMechanism(parent, driveMotor,
                new SimulatedBareEncoder(parent, driveMotor), 1);
        Profile100 profile = new TrapezoidProfile100(1, 1, 0.01);
        ZeroFeedback feedback = new ZeroFeedback(x -> x, 0.01, 0.01);
        ProfiledController controller = new IncrementalProfiledController(
                parent, profile, feedback, x -> x, 0.01, 0.01);
        return new OutboardGravityServo(
                parent,
                new OutboardAngularPositionServo(
                        parent,
                        rotaryMechanism,
                        new CombinedEncoder(
                                parent,
                                new SimulatedRotaryPositionSensor(parent, rotaryMechanism, () -> 0),
                                rotaryMechanism),
                        // true),
                        controller),
                1,
                1);
    }

    public static RotaryMechanism simulatedRotaryMechanism(LoggerFactory parent) {
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        return new SimpleRotaryMechanism(parent, driveMotor, new SimulatedBareEncoder(parent, driveMotor), 1);
    }

    public static LinearMechanism simulatedLinearMechanism(LoggerFactory parent) {
        // simulated drive motor free speed is 5 m/s
        SimulatedBareMotor driveMotor = new SimulatedBareMotor(parent, 5);
        // simulated gearing is 2 meter wheel, 1:1, so rad/s and m/s are the same.
        return new SimpleLinearMechanism(
                driveMotor,
                new SimulatedBareEncoder(parent, driveMotor),
                1,
                2);
    }
}
