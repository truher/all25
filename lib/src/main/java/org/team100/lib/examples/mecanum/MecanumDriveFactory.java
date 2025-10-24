package org.team100.lib.examples.mecanum;

import org.team100.lib.config.Identity;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.gyro.ReduxGyro;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanismFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.util.CanId;

public class MecanumDriveFactory {

    public static MecanumDrive100 make(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int supplyLimit,
            CanId gyroId,
            CanId frontLeft,
            CanId frontRight,
            CanId rearLeft,
            CanId rearRight,
            double gearRatio,
            double wheelDiaM) {
        return switch (Identity.instance) {
            case BLANK -> sim(
                    fieldLogger,
                    parent,
                    gearRatio,
                    wheelDiaM);
            default -> realRobot(
                    fieldLogger, parent, supplyLimit,
                    gyroId,
                    frontLeft,
                    frontRight,
                    rearLeft,
                    rearRight,
                    gearRatio,
                    wheelDiaM);
        };
    }

    private static MecanumDrive100 realRobot(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            int supplyLimit,
            CanId gyroId,
            CanId frontLeft,
            CanId frontRight,
            CanId rearLeft,
            CanId rearRight,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Mecanum Drive");
        
        // null gyro => use odometry for yaw.
        Gyro gyro = (gyroId == null) ? null : new ReduxGyro(log, gyroId);

        return new MecanumDrive100(
                fieldLogger,
                gyro,
                LinearMechanismFactory.neo(
                        log.name("frontLeft"),
                        supplyLimit,
                        frontLeft,
                        MotorPhase.REVERSE,
                        gearRatio,
                        wheelDiaM),
                LinearMechanismFactory.neo(
                        log.name("frontRight"),
                        supplyLimit,
                        frontRight,
                        MotorPhase.FORWARD,
                        gearRatio,
                        wheelDiaM),
                LinearMechanismFactory.neo(
                        log.name("rearLeft"),
                        supplyLimit,
                        rearLeft,
                        MotorPhase.REVERSE,
                        gearRatio,
                        wheelDiaM),
                LinearMechanismFactory.neo(
                        log.name("rearRight"),
                        supplyLimit,
                        rearRight,
                        MotorPhase.FORWARD,
                        gearRatio,
                        wheelDiaM));
    }

    private static MecanumDrive100 sim(
            LoggerFactory fieldLogger,
            LoggerFactory parent,
            double gearRatio,
            double wheelDiaM) {
        LoggerFactory log = parent.name("Mecanum Drive");

        // null gyro => use odometry for yaw
        return new MecanumDrive100(
                fieldLogger,
                null,
                LinearMechanismFactory.sim(
                        log.name("frontLeft"),
                        gearRatio,
                        wheelDiaM),
                LinearMechanismFactory.sim(
                        log.name("frontRight"),
                        gearRatio,
                        wheelDiaM),
                LinearMechanismFactory.sim(
                        log.name("rearLeft"),
                        gearRatio,
                        wheelDiaM),
                LinearMechanismFactory.sim(
                        log.name("rearRight"),
                        gearRatio,
                        wheelDiaM));
    }

}
