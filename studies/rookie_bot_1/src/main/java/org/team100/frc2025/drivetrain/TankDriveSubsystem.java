package org.team100.frc2025.drivetrain;

import org.team100.lib.encoder.IncrementalBareEncoder;
import org.team100.lib.encoder.NoEncoder;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.motion.servo.LinearVelocityServo;
import org.team100.lib.motion.servo.OutboardLinearVelocityServo;
import org.team100.lib.motor.MotorPhase;
import org.team100.lib.motor.TalonSRXMotor;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDriveSubsystem extends SubsystemBase {
    private static final double ROT_SCALE = 0.3;
    private static final double MAX_SPEED_M_S = 0.4;
    private static final double FIVE_TO_ONE = 5.2307692308;
    private static final double GEAR_RATIO = FIVE_TO_ONE * FIVE_TO_ONE;
    private static final double WHEEL_DIAM = 0.098425;

    private final LinearVelocityServo m_rearRight;
    private final LinearVelocityServo m_rearLeft;
    private final LinearVelocityServo m_frontRight;
    private final LinearVelocityServo m_frontLeft;

    public TankDriveSubsystem(LoggerFactory parent, int currentLimit) {
        LoggerFactory log = parent.type(this);
        m_rearLeft = makeServo("Rear Left", log, currentLimit, 5, MotorPhase.REVERSE);
        m_rearRight = makeServo("Rear Right", log, currentLimit, 10, MotorPhase.FORWARD);
        m_frontLeft = makeServo("Front Left", log, currentLimit, 11, MotorPhase.REVERSE);
        m_frontRight = makeServo("Front Right", log, currentLimit, 3, MotorPhase.FORWARD);
    }

    private static LinearVelocityServo makeServo(
            String name, LoggerFactory log, int currentLimit, int canId, MotorPhase phase) {
        TalonSRXMotor motor = new TalonSRXMotor(log, canId, phase, currentLimit);
        IncrementalBareEncoder encoder = new NoEncoder();
        LinearMechanism mech = new LinearMechanism(
                log, motor, encoder, GEAR_RATIO, WHEEL_DIAM,
                Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
        return new OutboardLinearVelocityServo(log, mech);
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
        setDutyCycle(s.left * MAX_SPEED_M_S, s.right * MAX_SPEED_M_S);
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
        m_rearLeft.setVelocityM_S(left);
        m_rearRight.setVelocityM_S(right);
        m_frontLeft.setVelocityM_S(left);
        m_frontRight.setVelocityM_S(right);
    }

    public void stop() {
        m_rearLeft.stop();
        m_rearRight.stop();
        m_frontLeft.stop();
        m_frontRight.stop();
    }

}
