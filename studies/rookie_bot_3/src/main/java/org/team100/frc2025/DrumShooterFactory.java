package org.team100.frc2025;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.NeutralMode;
import org.team100.lib.motor.rev.NeoVortexFactory;
import org.team100.lib.subsystems.shooter.DualDrumShooter;
import org.team100.lib.util.CanId;

/** Configuration of motors on the demobot shooter. */
public class DrumShooterFactory {
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIA_M = .1;

    public static DualDrumShooter make(LoggerFactory parent, int currentLimit) {
        LoggerFactory log = parent.name("shooter");
        LinearVelocityServo m_left = NeoVortexFactory.getNeoVortexVelocityServo(
                log.name("left"),
                currentLimit,
                new CanId(13),
                GEAR_RATIO,
                NeutralMode.BRAKE,
                MotorPhase.REVERSE,
                WHEEL_DIA_M);
        LinearVelocityServo m_right = NeoVortexFactory.getNeoVortexVelocityServo(
                log.name("right"),
                currentLimit,
                new CanId(40),
                GEAR_RATIO,
                NeutralMode.BRAKE,
                MotorPhase.FORWARD,
                WHEEL_DIA_M);

        return new DualDrumShooter(parent, m_left, m_right);
    }

}