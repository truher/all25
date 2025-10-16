package org.team100.lib.examples.tank;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.motor.BareMotor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Tank drive that uses two bare motors and provides a pose estimate using
 * odometry only.
 */
public class BareMotorTank extends SubsystemBase implements TankDrive {
    private static final double TRACK_WIDTH_M = 0.4;
    private static final double MAX_SPEED_M_S = 3.0;
    private static final double WHEEL_DIA_M = 0.15;
    private static final double WHEEL_RADIUS_M = 0.5 * WHEEL_DIA_M;
    // 12:72 gears
    private static final double GEAR_RATIO = 6.0;

    private final DoubleArrayLogger m_log_field_robot;
    private final BareMotor m_left;
    private final BareMotor m_right;
    private final DifferentialDriveKinematics m_kinematics;
    
    private double m_leftPosM;
    private double m_rightPosM;
    private Pose2d m_pose;

    public BareMotorTank(LoggerFactory fieldLogger, BareMotor left, BareMotor right) {
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, "robot");
        m_left = left;
        m_right = right;
        m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_M);
        m_leftPosM = 0;
        m_rightPosM = 0;
        m_pose = new Pose2d();
    }

    @Override
    public void setDutyCycle(double translationSpeed, double rotSpeed) {
        WheelSpeeds s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed, false);
        m_left.setDutyCycle(s.left);
        m_right.setDutyCycle(s.right);
    }

    @Override
    public void setVelocity(double translationM_S, double rotationRad_S) {
        ChassisSpeeds speed = new ChassisSpeeds(translationM_S, 0, rotationRad_S);
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speed);
        wheelSpeeds.desaturate(MAX_SPEED_M_S);
        m_left.setVelocity(wheelSpeeds.leftMetersPerSecond * GEAR_RATIO / WHEEL_RADIUS_M, 0, 0);
        m_right.setVelocity(wheelSpeeds.rightMetersPerSecond * GEAR_RATIO / WHEEL_RADIUS_M, 0, 0);
    }

    @Override
    public void stop() {
        m_left.stop();
        m_right.stop();
    }

    @Override
    public void periodic() {
        double leftM = m_left.getUnwrappedPositionRad() * WHEEL_RADIUS_M / GEAR_RATIO;
        double rightM = m_right.getUnwrappedPositionRad() * WHEEL_RADIUS_M / GEAR_RATIO;
        Twist2d twist = m_kinematics.toTwist2d(leftM - m_leftPosM, rightM - m_rightPosM);
        m_pose = m_pose.exp(twist);
        m_leftPosM = leftM;
        m_rightPosM = rightM;
        m_log_field_robot.log(this::poseArray);
        m_left.periodic();
        m_right.periodic();
    }

    private double[] poseArray() {
        return new double[] {
                m_pose.getX(),
                m_pose.getY(),
                m_pose.getRotation().getDegrees()
        };
    }

}
