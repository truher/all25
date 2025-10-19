package org.team100.lib.examples.mecanum;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.gyro.Gyro;
import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.DoubleArrayLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motor.BareMotor;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BareMotorMecanum extends SubsystemBase implements MecanumDrive {
    private static final double TRACK_WIDTH_M = 0.4;
    private static final double WHEELBASE_M = 0.4;
    private static final double MAX_SPEED_M_S = 3.0;
    private static final double WHEEL_DIA_M = 0.15;
    private static final double WHEEL_RADIUS_M = 0.5 * WHEEL_DIA_M;
    // 12:72 gears
    private static final double GEAR_RATIO = 6.0;
    private final DoubleArrayLogger m_log_field_robot;
    private final Gyro m_gyro;
    private final BareMotor m_frontLeft;
    private final BareMotor m_frontRight;
    private final BareMotor m_rearLeft;
    private final BareMotor m_rearRight;
    private final MecanumDriveKinematics m_kinematics;

    private Pose2d m_pose;

    public BareMotorMecanum(
            LoggerFactory fieldLogger,
            Gyro gyro,
            BareMotor frontLeft,
            BareMotor frontRight,
            BareMotor rearLeft,
            BareMotor rearRight) {
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
        m_pose = new Pose2d();
    }

    @Override
    public void setVelocity(GlobalVelocityR3 input) {
        // TODO: provide for gyro zero offset
        Rotation2d yaw = m_gyro.getYawNWU();
        ChassisSpeeds speed = SwerveKinodynamics.toInstantaneousChassisSpeeds(
                input, yaw);
        MecanumDriveWheelSpeeds mSpeed = m_kinematics.toWheelSpeeds(speed);
        m_frontLeft.setVelocity(mSpeed.frontLeftMetersPerSecond * GEAR_RATIO / WHEEL_RADIUS_M, 0, 0);
        m_frontRight.setVelocity(mSpeed.frontRightMetersPerSecond * GEAR_RATIO / WHEEL_RADIUS_M, 0, 0);
        m_rearLeft.setVelocity(mSpeed.rearLeftMetersPerSecond * GEAR_RATIO / WHEEL_RADIUS_M, 0, 0);
        m_rearRight.setVelocity(mSpeed.rearRightMetersPerSecond * GEAR_RATIO / WHEEL_RADIUS_M, 0, 0);
    }

    @Override
    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_rearLeft.stop();
        m_rearRight.stop();
    }

    @Override
    public void periodic() {
        // TODO: keep old positions, finish this
        Twist2d twist = m_kinematics.toTwist2d(null, null);
        m_pose = m_pose.exp(twist);
        m_log_field_robot.log(this::poseArray);
        m_frontLeft.periodic();
        m_frontRight.periodic();
        m_rearLeft.periodic();
        m_rearRight.periodic();
    }

    private double[] poseArray() {
        return new double[] {
                m_pose.getX(),
                m_pose.getY(),
                m_pose.getRotation().getDegrees()
        };
    }

}
