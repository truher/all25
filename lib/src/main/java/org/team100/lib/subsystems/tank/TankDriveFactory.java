package org.team100.lib.subsystems.tank;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.mechanism.LinearMechanismFactory;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.util.CanId;

public class TankDriveFactory {

    public static TankDrive make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int statorLimit,
            CanId canL,
            CanId canR,
            double trackWidthM,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Tank Drive");
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");
        LinearMechanismFactory f = new LinearMechanismFactory(
                statorLimit, gearRatio, wheelDiaM);
        return new TankDrive(fieldLogger, trackWidthM,
                new OutboardLinearVelocityServo(
                        logL, mech(f, logL, canL, MotorPhase.REVERSE)),
                new OutboardLinearVelocityServo(
                        logR, mech(f, logR, canR, MotorPhase.FORWARD)));
    }

    static LinearMechanism mech(
            LinearMechanismFactory f,
            LoggerFactory log,
            CanId can,
            MotorPhase phase) {
        return switch (Identity.instance) {
            case BLANK -> f.sim(log);
            default -> f.neo(log, can, phase);
        };
    }
}
