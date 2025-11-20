package org.team100.frc2025;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.BareMotor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.rev.Neo550CANSparkMotor;
import org.team100.lib.motor.rev.NeoVortexCANSparkMotor;
import org.team100.lib.servo.OutboardLinearVelocityServo;
import org.team100.lib.subsystems.shooter.SingleDrumShooter;
import org.team100.lib.util.CanId;

/** Configuration of motors on the demobot shooter. */
public class SingleShooterFactory {
    private static final double GEAR_RATIO = 1;
    private static final double WHEEL_DIA_M = .07;

    public static SingleDrumShooter make(
            LoggerFactory parent, int currentLimit) {
        LoggerFactory log = parent.name("shooter");

        CanId shooter = new CanId(7);

        Feedforward100 ff = Feedforward100.makeNeoVortex(log);
        // TODO: this P value is a guess
        PIDConstants pid = PIDConstants.makeVelocityPID(log, 0.0002);

        BareMotor motor = NeoVortexCANSparkMotor.get(log, shooter, MotorPhase.FORWARD, currentLimit, ff, pid);
        
        return new SingleDrumShooter(log,
                OutboardLinearVelocityServo.make(log, motor, GEAR_RATIO, WHEEL_DIA_M));
    }

}