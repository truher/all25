package org.team100.lib.examples.tank;


import org.team100.lib.motion.servo.LinearVelocityServo;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Tank drive that uses two linear velocity servos. */
public class ServoTank extends SubsystemBase implements TankDrive {

    private final LinearVelocityServo m_right;
    private final LinearVelocityServo m_left;

    public ServoTank(LinearVelocityServo left, LinearVelocityServo right) {
        m_left = left;
        m_right = right;
    }

    @Override
    public void set(double translationSpeed, double rotSpeed) {
        WheelSpeeds s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed, false);
        m_left.setVelocity(s.left);
        m_right.setVelocity(s.right);
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
