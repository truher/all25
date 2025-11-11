package org.team100.frc2025;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.rev.NeoVortexFactory;
import org.team100.lib.servo.LinearVelocityServo;
import org.team100.lib.servo.ServoFactory;
import org.team100.lib.subsystems.shooter.DualDrumShooter;
import org.team100.lib.util.CanId;

/** Configuration of motors on the demobot shooter. */
public class DrumShooterFactory {
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIA_M = .1;

    public static DualDrumShooter make(LoggerFactory parent, int currentLimit) {
        LoggerFactory log = parent.name("shooter");
        LinearVelocityServo m_left = servo(
                log.name("left"),
                currentLimit,
                new CanId(13),
                MotorPhase.REVERSE);
        LinearVelocityServo m_right = servo(
                log.name("right"),
                currentLimit,
                new CanId(40),
                MotorPhase.FORWARD);
        return new DualDrumShooter(log, m_left, m_right);
    }

    private static LinearVelocityServo servo(
            LoggerFactory log,
            int currentLimit,
            CanId canId,
            MotorPhase motorPhase) {
        switch (Identity.instance) {
            case BLANK -> {
                return ServoFactory.simulatedOutboardLinearVelocityServo(
                        log, GEAR_RATIO, WHEEL_DIA_M);
            }
            default -> {
                return NeoVortexFactory.getNeoVortexVelocityServo(
                        log,
                        currentLimit,
                        canId,
                        GEAR_RATIO,
                        NeutralMode.BRAKE,
                        motorPhase,
                        WHEEL_DIA_M);
            }
        }
    }

}