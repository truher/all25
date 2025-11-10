package org.team100.lib.subsystems.mecanum;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.mechanism.LinearMechanism;
import org.team100.lib.mechanism.LinearMechanismFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.sensor.gyro.Gyro;
import org.team100.lib.sensor.gyro.ReduxGyro;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.subsystems.mecanum.kinematics.MecanumKinematics100.Slip;
import org.team100.lib.util.CanId;

public class MecanumDriveFactory {

    public static MecanumDrive100 make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int statorLimit,
            CanId gyroId,
            CanId canFL,
            CanId canFR,
            CanId canRL,
            CanId canRR,
            double trackWidthM,
            double wheelbaseM,
            Slip slip,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Mecanum Drive");
        LoggerFactory logFL = log.name("frontLeft");
        LoggerFactory logFR = log.name("frontRight");
        LoggerFactory logRL = log.name("rearLeft");
        LoggerFactory logRR = log.name("rearRight");

        LinearMechanismFactory f = new LinearMechanismFactory(
                statorLimit, gearRatio, wheelDiaM);
        Gyro gyro = gyro(log, gyroId);
        slip = slip(slip);

        return new MecanumDrive100(log, fieldLogger, gyro, trackWidthM, wheelbaseM, slip,
                new OutboardLinearVelocityServo(
                        logFL, mech(f, logFL, canFL, MotorPhase.REVERSE)),
                new OutboardLinearVelocityServo(
                        logFR, mech(f, logFR, canFR, MotorPhase.FORWARD)),
                new OutboardLinearVelocityServo(
                        logRL, mech(f, logRL, canRL, MotorPhase.REVERSE)),
                new OutboardLinearVelocityServo(
                        logRR, mech(f, logRR, canRR, MotorPhase.FORWARD)));
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

    static Gyro gyro(LoggerFactory log, CanId gyroId) {
        if (gyroId == null)
            return null;
        return switch (Identity.instance) {
            case BLANK -> null;
            default -> new ReduxGyro(log, gyroId);
        };
    }

    static Slip slip(Slip slip) {
        return switch (Identity.instance) {
            case BLANK -> new Slip(1, 1, 1);// sim does not slip;
            default -> slip;
        };
    }

}
