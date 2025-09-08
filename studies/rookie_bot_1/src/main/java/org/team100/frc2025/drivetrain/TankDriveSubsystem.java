package org.team100.frc2025.drivetrain;

import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.TalonSRXMotor;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSubsystem extends SubsystemBase {
    private static final double ROT_SCALE = 0.3;
    private static final double SLOW = 0.4;

    private final TalonSRXMotor m_rearRight;
    private final TalonSRXMotor m_rearLeft;
    private final TalonSRXMotor m_frontRight;
    private final TalonSRXMotor m_frontLeft;

    public TankDriveSubsystem(LoggerFactory parent, int supplyLimit) {
        LoggerFactory log = parent.type(this);
        m_rearLeft = new TalonSRXMotor(log, 5, MotorPhase.REVERSE, supplyLimit);
        m_rearRight = new TalonSRXMotor(log, 10, MotorPhase.FORWARD, supplyLimit);
        m_frontLeft = new TalonSRXMotor(log, 11, MotorPhase.REVERSE, supplyLimit);
        m_frontRight = new TalonSRXMotor(log, 3, MotorPhase.FORWARD, supplyLimit);
    }

    @Override
    public void periodic() {
        m_rearLeft.periodic();
        m_rearRight.periodic();
        m_frontLeft.periodic();
        m_frontRight.periodic();
    }

    /**
     * @param translationSpeed -1 to 1
     * @param rotSpeed         -1 to 1
     */
    public void set(double translationSpeed, double rotSpeed) {
        WheelSpeeds s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed * ROT_SCALE, false);
        setDutyCycle(s.left * SLOW, s.right * SLOW);
    }

    /**
     * @param translationSpeed -1 to 1
     * @param rotSpeed         -1 to 1
     */
    public void setRaw(double translationSpeed, double rotSpeed) {
        WheelSpeeds s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed * ROT_SCALE, false);
        setDutyCycle(s.left, s.right);
    }

    public void setDutyCycle(double left, double right) {
        m_rearLeft.setDutyCycle(left);
        m_rearRight.setDutyCycle(right);
        m_frontLeft.setDutyCycle(left);
        m_frontRight.setDutyCycle(right);
    }

    public void stop() {
        m_rearLeft.stop();
        m_rearRight.stop();
        m_frontLeft.stop();
        m_frontRight.stop();
    }

}
