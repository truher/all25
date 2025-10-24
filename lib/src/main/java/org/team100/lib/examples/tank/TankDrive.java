package org.team100.lib.examples.tank;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.motion.mechanism.LinearMechanism;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Tank drive that uses two linear mechanisms and provides a pose estimate using
 * odometry only.
 */
public class TankDrive extends SubsystemBase {
    private static final double TRACK_WIDTH_M = 0.4;
    private static final double MAX_SPEED_M_S = 3.0;

    private final DoubleArrayLogger m_log_field_robot;
    private final LinearMechanism m_left;
    private final LinearMechanism m_right;
    private final DifferentialDriveKinematics m_kinematics;
    private DifferentialDriveWheelPositions m_positions;
    private Pose2d m_pose;

    public TankDrive(
            LoggerFactory fieldLogger,
            LinearMechanism left,
            LinearMechanism right) {
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, "robot");
        m_left = left;
        m_right = right;
        m_kinematics = new DifferentialDriveKinematics(TRACK_WIDTH_M);
        m_positions = new DifferentialDriveWheelPositions(0, 0);
        m_pose = new Pose2d();
    }

    /** Use arcade drive to set duty cycle directly. */
    public void setDutyCycle(double translationSpeed, double rotSpeed) {
        WheelSpeeds s = DifferentialDrive.arcadeDriveIK(
                translationSpeed, rotSpeed, false);
        m_left.setDutyCycle(s.left);
        m_right.setDutyCycle(s.right);
    }

    /** Use inverse kinematics to set wheel speeds. */
    public void setVelocity(double translationM_S, double rotationRad_S) {
        ChassisSpeeds speed = new ChassisSpeeds(translationM_S, 0, rotationRad_S);
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speed);
        wheelSpeeds.desaturate(MAX_SPEED_M_S);
        m_left.setVelocity(wheelSpeeds.leftMetersPerSecond, 0, 0);
        m_right.setVelocity(wheelSpeeds.rightMetersPerSecond, 0, 0);
    }

    public void stop() {
        m_left.stop();
        m_right.stop();
    }

    @Override
    public void periodic() {
        updatePose();
        m_log_field_robot.log(this::poseArray);
        m_left.periodic();
        m_right.periodic();
    }

    public void setPose(Pose2d p) {
        m_pose = p;
    }

    public Pose2d getPose() {
        return m_pose;
    }

    /** Set the drive velocity. */
    public Command driveWithVelocity(double translationM_S, double rotationRad_s) {
        return run(() -> setVelocity(translationM_S, rotationRad_s))
                .withName("drive with velocity");
    }

    private void updatePose() {
        Twist2d twist = twist();
        m_pose = m_pose.exp(twist);
    }

    private Twist2d twist() {
        DifferentialDriveWheelPositions newPositions = new DifferentialDriveWheelPositions(
                m_left.getPositionM(),
                m_right.getPositionM());
        Twist2d twist = m_kinematics.toTwist2d(m_positions, newPositions);
        m_positions = newPositions;
        return twist;
    }

    private double[] poseArray() {
        return new double[] {
                m_pose.getX(),
                m_pose.getY(),
                m_pose.getRotation().getDegrees()
        };
    }

}
