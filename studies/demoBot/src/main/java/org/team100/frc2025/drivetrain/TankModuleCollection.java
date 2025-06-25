package org.team100.frc2025.drivetrain;

import java.util.OptionalDouble;

import org.team100.lib.config.Identity;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.Neo550Factory;
import org.team100.lib.util.Util;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class TankModuleCollection {
    private static final String kSwerveModules = "Tank Modules";
    private static final String kLeft = "Left";
    private static final String kRight = "Right";
    private static final double m_5to1 = 5.2307692308;
    private static final double m_wheelDiameter = 0.098425;

    private final VictorSP m_rightVictorSP;
    private final VictorSP m_leftVictorSP;
    private final OutboardLinearVelocityServo m_rightDrive;
    private final OutboardLinearVelocityServo m_leftDrive;

    private TankModuleCollection(OutboardLinearVelocityServo leftDrive, OutboardLinearVelocityServo rightDrive) {
        m_leftDrive = leftDrive;
        m_rightDrive = rightDrive;
        m_rightVictorSP = null;
        m_leftVictorSP = null;
    }

    private TankModuleCollection(VictorSP leftDrive, VictorSP rightDrive) {
        m_leftDrive = null;
        m_rightDrive = null;
        m_leftVictorSP = leftDrive;
        m_rightVictorSP = rightDrive;
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
                 VictorSP rightVictorSP = new VictorSP(8);
                VictorSP leftVictorSP = new VictorSP(9);
                return new TankModuleCollection(leftVictorSP, rightVictorSP);
            case BLANK:
                Util.println("************** SIMULATED MODULES **************");
            default:
                return new TankModuleCollection(
                        Neo550Factory.simulatedDriveServo(collectionLogger.child(kLeft)),
                        Neo550Factory.simulatedDriveServo(collectionLogger.child(kRight)));
        }
    }

    public void setDrive(double[] value) {
        if (m_rightVictorSP == null) {
            m_leftDrive.setVelocityM_S(value[0]);
            m_rightDrive.setVelocityM_S(value[1]);
        } else {
            m_leftVictorSP.set(-1.0 * value[0]);
            m_rightVictorSP.set(value[1]);
        }
    }

    public OptionalDouble[] getSpeeds() {
        if (m_rightVictorSP == null) {
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

    public void stop() {
        if (m_rightVictorSP == null) {
            m_leftDrive.stop();
            m_rightDrive.stop();
        } else {
            m_leftVictorSP.set(0);
            m_rightVictorSP.set(0);
        }
    }
}
