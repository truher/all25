package org.team100.frc2025.drivetrain;

import java.util.OptionalDouble;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550Factory;
import org.team100.lib.util.Util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TankModuleCollection {
    private static final String kSwerveModules = "Tank Modules";
    private static final String kLeft = "Left";
    private static final String kRight = "Right";
    private static final double m_5to1 = 5.2307692308;
    private static final double m_wheelDiameter = 0.098425;

    private final TalonSRX m_rearRightTalonSRX;
    private final TalonSRX m_rearLeftTalonSRX;
    private final TalonSRX m_frontRightTalonSRX;
    private final TalonSRX m_frontLeftTalonSRX;
    private final OutboardLinearVelocityServo m_rightDrive;
    private final OutboardLinearVelocityServo m_leftDrive;

    private TankModuleCollection(OutboardLinearVelocityServo leftDrive, OutboardLinearVelocityServo rightDrive) {
        m_leftDrive = leftDrive;
        m_rightDrive = rightDrive;
        m_rearRightTalonSRX = null;
        m_rearLeftTalonSRX = null;
        m_frontRightTalonSRX = null;
        m_frontLeftTalonSRX = null;
    }

    private TankModuleCollection(TalonSRX rearLeftDrive, TalonSRX rearRightDrive, TalonSRX frontLeftDrive,
            TalonSRX frontRightDrive) {
        m_leftDrive = null;
        m_rightDrive = null;
        m_rearLeftTalonSRX = rearLeftDrive;
        m_rearRightTalonSRX = rearRightDrive;
        m_frontLeftTalonSRX = frontLeftDrive;
        m_frontRightTalonSRX = frontRightDrive;
    }

    /**
     * Creates collections according to Identity.
     */
    public static TankModuleCollection get(
            LoggerFactory parent,
            int currentLimit) {
        LoggerFactory collectionLogger = parent.child(kSwerveModules);
        switch (Identity.instance) {
            case DEMO_BOT:
                Util.println("************** Custom Tank Module Modules, using in built encoders? **************");
                LinearMechanism rightMechanism = Neo550Factory.getNEO550LinearMechanism(
                        kRight, collectionLogger, currentLimit, 2, Math.pow(m_5to1, 2), MotorPhase.REVERSE,
                        m_wheelDiameter);
                OutboardLinearVelocityServo rightMotor = new OutboardLinearVelocityServo(collectionLogger,
                        rightMechanism);
                LinearMechanism leftMechanism = Neo550Factory.getNEO550LinearMechanism(
                        kLeft, collectionLogger, currentLimit, 3, Math.pow(m_5to1, 2), MotorPhase.FORWARD,
                        m_wheelDiameter);
                OutboardLinearVelocityServo leftMotor = new OutboardLinearVelocityServo(collectionLogger,
                        leftMechanism);
                return new TankModuleCollection(leftMotor, rightMotor);
            case ROOKIE_BOT:
                TalonSRX rearRightTalonSRX = new TalonSRX(10);
                TalonSRX rearLeftTalonSRX = new TalonSRX(5);
                rearLeftTalonSRX.setInverted(true);
                TalonSRX frontRightTalonSRX = new TalonSRX(3);
                TalonSRX frontLeftTalonSRX = new TalonSRX(11);
                frontLeftTalonSRX.setInverted(true);
                return new TankModuleCollection(rearLeftTalonSRX, rearRightTalonSRX, frontLeftTalonSRX,
                        frontRightTalonSRX);
            case BLANK:
                Util.println("************** SIMULATED MODULES **************");
            default:
                return new TankModuleCollection(
                        Neo550Factory.simulatedDriveServo(collectionLogger.child(kLeft)),
                        Neo550Factory.simulatedDriveServo(collectionLogger.child(kRight)));
        }
    }

    public void setDrive(double[] value) {
        if (m_rearRightTalonSRX == null) {
            m_leftDrive.setVelocityM_S(value[0]);
            m_rightDrive.setVelocityM_S(value[1]);
        } else {
            m_rearLeftTalonSRX.set(ControlMode.PercentOutput, value[0]);
            m_rearRightTalonSRX.set(ControlMode.PercentOutput, value[1]);
            m_frontLeftTalonSRX.set(ControlMode.PercentOutput, value[0]);
            m_frontRightTalonSRX.set(ControlMode.PercentOutput, value[1]);
        }
    }

    public OptionalDouble[] getSpeeds() {
        if (m_rearRightTalonSRX == null) {
            return new OptionalDouble[] {
                    m_leftDrive.getVelocity(),
                    m_rightDrive.getVelocity()
            };
        } else {
            return new OptionalDouble[] {
                    OptionalDouble.empty(),
                    OptionalDouble.empty()
            };
        }
    }

    public OptionalDouble[] getCurrent() {
        if (m_rearRightTalonSRX == null) {
            return new OptionalDouble[] {
                    OptionalDouble.empty(),
                    OptionalDouble.empty(),
                    OptionalDouble.empty(),
                    OptionalDouble.empty()
            };
        } else {
            return new OptionalDouble[] {
                    OptionalDouble.of(m_rearLeftTalonSRX.getSupplyCurrent()),
                    OptionalDouble.of(m_rearRightTalonSRX.getSupplyCurrent()),
                    OptionalDouble.of(m_frontLeftTalonSRX.getSupplyCurrent()),
                    OptionalDouble.of(m_frontRightTalonSRX.getSupplyCurrent())
            };
        }
    }

    public void stop() {
        if (m_rearRightTalonSRX == null) {
            m_leftDrive.stop();
            m_rightDrive.stop();
        } else {
            m_rearLeftTalonSRX.set(ControlMode.PercentOutput, 0);
            m_rearRightTalonSRX.set(ControlMode.PercentOutput, 0);
            m_frontLeftTalonSRX.set(ControlMode.PercentOutput, 0);
            m_frontRightTalonSRX.set(ControlMode.PercentOutput, 0);
        }
    }
}
