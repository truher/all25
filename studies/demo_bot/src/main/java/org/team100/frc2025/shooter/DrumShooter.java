package org.team100.frc2025.shooter;

import java.util.OptionalDouble;

import org.ejml.simple.UnsupportedOperation;
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

    private final DoubleLogger m_leftlogger;
    private final DoubleLogger m_rightlogger;
    private final BooleanLogger m_atVelocitylogger;

    private final LinearVelocityServo m_leftRoller;
    private final LinearVelocityServo m_rightRoller;
    private final double shooterVelocityM_S = 10;

    private double currentDesiredLeftVelocity = 0;
    private double currentDesiredRightVelocity = 0;

    public DrumShooter(LoggerFactory parent, int currentLimit) {

        LoggerFactory collectionLogger = parent.type(this);

        // TODO get the real diameter, gearRatios, and canIDs, and Indexer accel

        m_leftRoller = Neo550Factory.getNEO550VelocityServo("Left", collectionLogger,
                currentLimit, 39, GEAR_RATIO, MotorPhase.FORWARD, WHEEL_DIA_M);
        m_rightRoller = Neo550Factory.getNEO550VelocityServo("Right", collectionLogger,
                currentLimit, 19, GEAR_RATIO, MotorPhase.REVERSE, WHEEL_DIA_M);

        LoggerFactory logger = parent.type(this);
        m_atVelocitylogger = logger.booleanLogger(Level.TRACE, "At velocity");
        m_leftlogger = logger.doubleLogger(Level.TRACE, "Left Shooter Desired");
        m_rightlogger = logger.doubleLogger(Level.TRACE, "Right Shooter Desired");

    }

    public void set(double velocityM_S) {
        m_leftRoller.setVelocityM_S(velocityM_S);
        m_rightRoller.setVelocityM_S(velocityM_S);
        currentDesiredLeftVelocity = velocityM_S;
        currentDesiredRightVelocity = velocityM_S;
        m_leftlogger.log(() -> velocityM_S);
        m_rightlogger.log(() -> velocityM_S);
    }

    public void stop() {
        set(0);
    }

    public void setIndividual(double leftVelocityM_S, double rightVelocityM_S) {
        m_leftRoller.setVelocityM_S(leftVelocityM_S);
        m_rightRoller.setVelocityM_S(rightVelocityM_S);
        currentDesiredLeftVelocity = leftVelocityM_S;
        currentDesiredRightVelocity = rightVelocityM_S;
        m_leftlogger.log(() -> rightVelocityM_S);
        m_rightlogger.log(() -> leftVelocityM_S);
    }

    public void spinUp() {
        set(shooterVelocityM_S);
    }

    /** Returns the average of the two rollers */
    public double getVelocity() {
        return (getLeftVelocityM_S() + getRightVelocityM_S()) / 2;
    }

    public boolean atVelocity() {
        return atVelocity(10);
    }

    /**
     * @param tolerance Units are M_S
     * @return If the absolute value of the error is less than the tolerance
     */
    public boolean atVelocity(double tolerance) {
        return Math.abs(rightError()) < tolerance && Math.abs(leftError()) < tolerance;
    }

    public double rightError() {
        return getRightVelocityM_S() - currentDesiredRightVelocity;
    }

    public double leftError() {
        return getLeftVelocityM_S() - currentDesiredLeftVelocity;
    }

    public double getLeftVelocityM_S() {
        return m_leftRoller.getVelocity().orElseThrow();
    }

    public double getRightVelocityM_S() {
        return m_rightRoller.getVelocity().orElseThrow();
    }

    @Override
    public void periodic() {
        m_leftRoller.periodic();
        m_rightRoller.periodic();
        m_atVelocitylogger.log(this::atVelocity);
    }
}