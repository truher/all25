package org.team100.lib.examples.tank;

import org.team100.lib.motor.BareMotor;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Tank drive that uses two bare motor groups.
 */
public class BareMotorTank extends SubsystemBase implements TankDrive {

    private final BareMotor m_left;
    private final BareMotor m_right;

    public BareMotorTank(BareMotor left, BareMotor right) {
        m_left = left;
        m_right = right;
    }

    @Override
    public void set(double translationSpeed, double rotSpeed) {
        WheelSpeeds s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed, false);
        m_left.setDutyCycle(s.left);
        m_right.setDutyCycle(s.right);
    }

    @Override
    public void stop() {
        m_left.stop();
        m_right.stop();
    }

    @Override
    public void periodic() {
        m_left.periodic();
        m_right.periodic();
    }
}
