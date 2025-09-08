package org.team100.frc2025.shooter;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.BooleanLogger;
import org.team100.lib.logging.LoggerFactory.DoubleLogger;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550Factory;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Direct-drive shooter with left and right drums.
 * 
 * Typical free speed of 6k rpm => 100 turn/sec
 * diameter of 0.1m => 0.314 m/turn
 * therefore top speed is around 30 m/s.
 * 
 * Empirically it seems to take a second or so to spin
 * up, so set the acceleration a bit higher than that to start.
 */
public class DrumShooter extends SubsystemBase {
    private static final double GEAR_RATIO = 5.2307692308;
    private static final double WHEEL_DIA_M = .33;
    private static final double TARGET_VELOCITY_M_S = 10;

    private final LinearVelocityServo m_left;
    private final LinearVelocityServo m_right;
    private final BooleanLogger m_log_atGoal;

    public DrumShooter(LoggerFactory parent, int currentLimit) {
        LoggerFactory collectionLogger = parent.type(this);
        m_left = Neo550Factory.getNEO550VelocityServo(
                "Left", collectionLogger, currentLimit, 39,
                GEAR_RATIO, MotorPhase.FORWARD, WHEEL_DIA_M);
        m_right = Neo550Factory.getNEO550VelocityServo(
                "Right", collectionLogger, currentLimit, 19,
                GEAR_RATIO, MotorPhase.REVERSE, WHEEL_DIA_M);
        LoggerFactory logger = parent.type(this);
        m_log_atGoal = logger.booleanLogger(Level.TRACE, "At goal");
    }

    public void set(double velocityM_S) {
        m_left.setVelocity(velocityM_S);
        m_right.setVelocity(velocityM_S);
    }

    public void stop() {
        set(0);
    }

    public void spinUp() {
        set(TARGET_VELOCITY_M_S);
    }

    public boolean atGoal() {
        return m_right.atGoal() && m_left.atGoal();
    }

    @Override
    public void periodic() {
        m_left.periodic();
        m_right.periodic();
        m_log_atGoal.log(this::atGoal);
    }
}