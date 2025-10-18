package org.team100.lib.examples.tank;

import org.team100.lib.motion.servo.LinearVelocityServo;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Tank drive that uses two linear velocity servos. */
public class ServoTank extends SubsystemBase implements TankDrive {
    private static final double TRACK_WIDTH_M = 0.4;
    private static final double MAX_SPEED_M_S = 3.0;
    private final LinearVelocityServo m_right;
    private final LinearVelocityServo m_left;
    private final DifferentialDriveKinematics m_kinematics;

    public ServoTank(LinearVelocityServo left, LinearVelocityServo right) {
        m_left = left;
        m_right = right;
        m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_M);
    }

    @Override
    public void setDutyCycle(double translationSpeed, double rotSpeed) {
        WheelSpeeds s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed, false);
        m_left.setVelocity(s.left);
        m_right.setVelocity(s.right);
    }

    @Override
    public void setVelocity(double translationM_S, double rotationRad_S) {
        ChassisSpeeds speed = new ChassisSpeeds(translationM_S, 0, rotationRad_S);
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speed);
        wheelSpeeds.desaturate(MAX_SPEED_M_S);
        m_left.setVelocity(wheelSpeeds.leftMetersPerSecond, 0);
        m_right.setVelocity(wheelSpeeds.rightMetersPerSecond, 0);
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
