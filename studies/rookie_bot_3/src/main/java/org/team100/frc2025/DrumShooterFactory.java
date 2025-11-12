package org.team100.frc2025;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.subsystems.shooter.DualDrumShooter;
import org.team100.lib.util.CanId;

/** Configuration of motors on the demobot shooter. */
public class DrumShooterFactory {
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIA_M = .1;

    public static DualDrumShooter make(LoggerFactory parent, int currentLimit) {
        LoggerFactory log = parent.name("shooter");
        LoggerFactory logL = log.name("left");
        LoggerFactory logR = log.name("right");

        CanId canL = new CanId(13);
        CanId canR = new CanId(40);

        Feedforward100 ff = Feedforward100.makeNeoVortex(log);
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.00025, 0, 0.0001);

        BareMotor motorL = NeoVortexCANSparkMotor.get(log, canL, MotorPhase.REVERSE, currentLimit, ff, pid);
        BareMotor motorR = NeoVortexCANSparkMotor.get(log, canR, MotorPhase.FORWARD, currentLimit, ff, pid);

        return new DualDrumShooter(log,
                OutboardLinearVelocityServo.make(logL, motorL, GEAR_RATIO, WHEEL_DIA_M),
                OutboardLinearVelocityServo.make(logR, motorR, GEAR_RATIO, WHEEL_DIA_M));
    }
}