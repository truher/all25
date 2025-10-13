package org.team100.frc2025;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.CANSparkEncoder;
import org.team100.lib.examples.tank.ServoTank;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550CANSparkMotor;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.util.CanId;

/** Configuration of motors on the Demo Bot. */
public class TankFactory {
    private static final double FIVE_TO_ONE = 5.2307692308;
    private static final double GEAR_RATIO = FIVE_TO_ONE * FIVE_TO_ONE;
    private static final double WHEEL_DIAM = 0.098425;

    public static ServoTank make(LoggerFactory parent, int currentLimit) {
        LoggerFactory log = parent.type("Tank Drive");
        LinearVelocityServo left = makeServo(log.name("left"), currentLimit, new CanId(3));
        LinearVelocityServo right = makeServo(log.name("right"), currentLimit, new CanId(2));
        return new ServoTank(left, right);
    }

    public static LinearVelocityServo makeServo(
            LoggerFactory log, int currentLimit, CanId canId) {
        Neo550CANSparkMotor motor = new Neo550CANSparkMotor(
                log,
                canId,
                NeutralMode.BRAKE,
                MotorPhase.FORWARD,
                currentLimit,
                Feedforward100.makeNeo550(),
                new PIDConstants());
        CANSparkEncoder encoder = new CANSparkEncoder(log, motor);
        LinearMechanism mech = new LinearMechanism(
                log, motor, encoder, GEAR_RATIO, WHEEL_DIAM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(log, mech);
    }

}
