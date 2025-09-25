package org.team100.frc2025.Wrist;

import org.team100.lib.config.Feedforward100;
import org.team100.lib.config.Identity;
import org.team100.lib.config.PIDConstants;
import org.team100.lib.encoder.SimulatedBareEncoder;
import org.team100.lib.encoder.Talon6Encoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.Kraken6Motor;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.SimulatedBareMotor;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {

    private final LinearMechanism m_leftMotor;
    private final LinearMechanism m_rightMotor;
    private final LinearMechanism m_algaeMotor;
    private final LaserCan m_coralRightLaserCan;
    private final LaserCan m_coralCenterLaserCan;
    private final LaserCan m_coralBackLaserCan;
    private final LaserCan m_coralLeftLaserCan;

    public Manipulator(LoggerFactory log) {
        switch (Identity.instance) {
            case COMP_BOT:
                // Set specific parameters for the competition robot
                Kraken6Motor m_KrakenLeftMotor = new Kraken6Motor(log, 54, MotorPhase.FORWARD, 40, 40,
                        new PIDConstants(), Feedforward100.makeShooterFalcon6());
                Kraken6Motor m_KrakenRightMotor = new Kraken6Motor(log, 55, MotorPhase.REVERSE, 40, 40,
                        new PIDConstants(), Feedforward100.makeShooterFalcon6());
                Kraken6Motor m_KrakenAlgaeMotor = new Kraken6Motor(log, 56, MotorPhase.FORWARD, 120, 120,
                        new PIDConstants(), Feedforward100.makeShooterFalcon6());
                m_coralRightLaserCan = new LaserCan(52);
                m_coralCenterLaserCan = new LaserCan(51);
                m_coralBackLaserCan = new LaserCan(53);
                m_coralLeftLaserCan = new LaserCan(50);
                m_leftMotor = new LinearMechanism(log, m_KrakenLeftMotor, new Talon6Encoder(log, m_KrakenLeftMotor), 16,
                        .1, -100000000, 1000000);
                m_rightMotor = new LinearMechanism(log, m_KrakenRightMotor, new Talon6Encoder(log, m_KrakenRightMotor),
                        16, .1, -100000000, 1000000);
                m_algaeMotor = new LinearMechanism(log, m_KrakenAlgaeMotor, new Talon6Encoder(log, m_KrakenAlgaeMotor),
                        16, .1, -100000000, 1000000);

                break;

            default:
                SimulatedBareMotor leftMotor = new SimulatedBareMotor(log, 100);
                SimulatedBareEncoder leftEncoder = new SimulatedBareEncoder(log, leftMotor);
                SimulatedBareMotor rightMotor = new SimulatedBareMotor(log, 100);
                SimulatedBareEncoder rightEncoder = new SimulatedBareEncoder(log, rightMotor);
                SimulatedBareMotor algaeMotor = new SimulatedBareMotor(log, 100);
                SimulatedBareEncoder algaeEncoder = new SimulatedBareEncoder(log, algaeMotor);
                m_leftMotor = new LinearMechanism(
                        log, leftMotor, leftEncoder, 1, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_rightMotor = new LinearMechanism(
                        log, rightMotor, rightEncoder, 1, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_algaeMotor = new LinearMechanism(
                        log, algaeMotor, algaeEncoder, 1, 1,
                        Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
                m_coralRightLaserCan = new LaserCan(52);
                m_coralCenterLaserCan = new LaserCan(51);
                m_coralBackLaserCan = new LaserCan(53);
                m_coralLeftLaserCan = new LaserCan(50);
                break;
        }
    }

    public boolean isCoralClose(double distance) {
        return distance < 100;
    }

    public void intakeCenter() {
        if (isCoralClose(m_coralBackLaserCan.getMeasurement().distance_mm)) {
            stop();
        } else {
            m_algaeMotor.setDutyCycle(-0.5);
            m_leftMotor.setDutyCycle(0.5);
            m_rightMotor.setDutyCycle(0.5);
        }
    }

    public void intakeSideways() {
        if (isCoralClose(m_coralLeftLaserCan.getMeasurement().distance_mm)
                && isCoralClose(m_coralRightLaserCan.getMeasurement().distance_mm)) {
            stop();
        } else {
            m_algaeMotor.setDutyCycle(-0.5);
            if (isCoralClose(m_coralLeftLaserCan.getMeasurement().distance_mm)) {
                m_leftMotor.setDutyCycle(0.5);
                m_rightMotor.setDutyCycle(-0.5);
            } else {
                m_leftMotor.setDutyCycle(-0.5);
                m_rightMotor.setDutyCycle(0.5);
            }
        }
    }

    public void stop() {
        m_algaeMotor.setDutyCycle(0);
        m_leftMotor.setDutyCycle(0);
        m_rightMotor.setDutyCycle(0);
    }

    public void intakeAlgae() {
//         if (m_algaeMotor.getCurrent() > 80) {
//             m_algaeMotor.setDutyCycle(0.5);
//         } else {
//             m_algaeMotor.setDutyCycle(1);
//         }
        m_algaeMotor.setDutyCycle(1);
    }

}
