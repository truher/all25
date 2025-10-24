package org.team100.lib.examples.mecanum;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.mechanism.LinearMechanism;
import org.team100.lib.state.ModelR3;
import org.team100.lib.subsystems.SubsystemR3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Mecanum drive with optional gyro. */
public class MecanumDrive100 extends SubsystemBase implements SubsystemR3 {
    private static final double TRACK_WIDTH_M = 0.4;
    private static final double WHEELBASE_M = 0.4;

    private final DoubleArrayLogger m_log_field_robot;
    /** May be null. */
    private final Gyro m_gyro;
    private final LinearMechanism m_frontLeft;
    private final LinearMechanism m_frontRight;
    private final LinearMechanism m_rearLeft;
    private final LinearMechanism m_rearRight;
    private final MecanumDriveKinematics m_kinematics;

    private MecanumDriveWheelPositions m_positions;
    private GlobalVelocityR3 m_input;
    private Pose2d m_pose;
    private Rotation2d m_gyroOffset;

    /**
     * Gyro may be null, in which case we use (not very accurate) odometry for yaw.
     */
    public MecanumDrive100(
            LoggerFactory fieldLogger,
            Gyro gyro,
            LinearMechanism frontLeft,
            LinearMechanism frontRight,
            LinearMechanism rearLeft,
            LinearMechanism rearRight) {
        m_log_field_robot = fieldLogger.doubleArrayLogger(Level.COMP, "robot");
        m_gyro = gyro;
        m_frontLeft = frontLeft;
        m_frontRight = frontRight;
        m_rearLeft = rearLeft;
        m_rearRight = rearRight;
        m_kinematics = new MecanumDriveKinematics(
                new Translation2d(WHEELBASE_M / 2, TRACK_WIDTH_M / 2),
                new Translation2d(WHEELBASE_M / 2, -TRACK_WIDTH_M / 2),
                new Translation2d(-WHEELBASE_M / 2, TRACK_WIDTH_M / 2),
                new Translation2d(-WHEELBASE_M / 2, -TRACK_WIDTH_M / 2));
        m_positions = new MecanumDriveWheelPositions();
        m_input = new GlobalVelocityR3(0, 0, 0);
        m_pose = new Pose2d();
        m_gyroOffset = new Rotation2d();
    }

    @Override
    public ModelR3 getState() {
        // assume the velocity is exactly what was requested.
        return new ModelR3(m_pose, m_input);
    }

    /** Use inverse kinematics to set wheel speeds. */
    @Override
    public void setVelocity(GlobalVelocityR3 input) {
        Rotation2d yaw = getYaw();
        ChassisSpeeds speed = SwerveKinodynamics.toInstantaneousChassisSpeeds(
                input, yaw);
        MecanumDriveWheelSpeeds mSpeed = m_kinematics.toWheelSpeeds(speed);
        m_frontLeft.setVelocity(mSpeed.frontLeftMetersPerSecond, 0, 0);
        m_frontRight.setVelocity(mSpeed.frontRightMetersPerSecond, 0, 0);
        m_rearLeft.setVelocity(mSpeed.rearLeftMetersPerSecond, 0, 0);
        m_rearRight.setVelocity(mSpeed.rearRightMetersPerSecond, 0, 0);
    }

    private Rotation2d getYaw() {
        if (m_gyro == null)
            return m_pose.getRotation();
        return m_gyro.getYawNWU().minus(m_gyroOffset);
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    /** Set the field-relative velocity. */
    public Command driveWithGlobalVelocity(GlobalVelocityR3 v) {
        return run(() -> setVelocity(v))
                .withName("drive with global velocity");
    }

    public Command resetPose() {
        return runOnce(() -> m_pose = new Pose2d());
    }

    /** Set yaw to zero. */
    public Command resetYaw() {
        return runOnce(this::resetGyroOffset);
    }

    @Override
    public void periodic() {
        updatePose();
        m_log_field_robot.log(this::poseArray);
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
    }

    private void resetGyroOffset() {
        if (m_gyro == null)
            return;
        m_gyroOffset = m_gyro.getYawNWU();
    }

    private void updatePose() {
        Twist2d twist = twist();
        m_pose = m_pose.exp(twist);
    }

    private Twist2d twist() {
        MecanumDriveWheelPositions newPositions = new MecanumDriveWheelPositions(
                m_frontLeft.getPositionM(),
                m_frontRight.getPositionM(),
                m_rearLeft.getPositionM(),
                m_rearRight.getPositionM());
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
