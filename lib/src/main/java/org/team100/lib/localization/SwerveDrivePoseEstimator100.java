package org.team100.lib.localization;

import java.util.Map.Entry;
import java.util.SortedMap;

import org.team100.lib.logging.Level;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.logging.LoggerFactory.Rotation2dLogger;
import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.motion.drivetrain.state.SwerveModulePositions;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveDrivePoseEstimator100 implements PoseEstimator100 {
    /**
     * The buffer only needs to be long enough to catch stale-but-still-helpful
     * vision updates.
     * 
     * The current Raspberry Pi cameras seem to be able to provide frames to RoboRIO
     * code with about 75-100 ms latency. There will never be a vision update
     * older than about 200 ms.
     */
    private static final double BUFFER_DURATION = 0.2;

    private final SwerveKinodynamics m_kinodynamics;
    private final TimeInterpolatableBuffer100<InterpolationRecord> m_poseBuffer;

    private final Rotation2dLogger m_log_offset;

    /**
     * maintained in resetPosition().
     */
    private Rotation2d m_gyroOffset;

    /**
     * @param kinodynamics      A correctly-configured kinodynamics object
     *                          for your drivetrain.
     * @param gyroAngle         The current gyro angle.
     * @param modulePositions   The current distance and rotation
     *                          measurements of the swerve modules.
     * @param initialPoseMeters The starting pose estimate.
     */
    public SwerveDrivePoseEstimator100(
            LoggerFactory parent,
            SwerveKinodynamics kinodynamics,
            Rotation2d gyroAngle,
            SwerveModulePositions modulePositions,
            Pose2d initialPoseMeters,
            double timestampSeconds) {
        LoggerFactory child = parent.type(this);
        m_kinodynamics = kinodynamics;
        m_poseBuffer = new TimeInterpolatableBuffer100<>(
                BUFFER_DURATION,
                timestampSeconds,
                new InterpolationRecord(
                        m_kinodynamics.getKinematics(),
                        new SwerveModel(
                                initialPoseMeters,
                                new FieldRelativeVelocity(0, 0, 0)),
                        modulePositions));
        m_gyroOffset = initialPoseMeters.getRotation().minus(gyroAngle);
        m_log_offset = child.rotation2dLogger(Level.TRACE, "GYRO OFFSET");
    }

    /**
     * timestamp in seconds
     */
    public void put(
            double timestamp,
            SwerveModel model,
            SwerveModulePositions positions) {
        m_poseBuffer.put(
                timestamp,
                new InterpolationRecord(
                        m_kinodynamics.getKinematics(),
                        model,
                        positions));
    }

    public Entry<Double, InterpolationRecord> lowerEntry(double timestamp) {
        return m_poseBuffer.lowerEntry(timestamp);
    }

    public InterpolationRecord getRecord(double timestamp) {
        return m_poseBuffer.get(timestamp);
    }

    public boolean tooOld(double timestamp) {
        return m_poseBuffer.tooOld(timestamp);
    }

    public SortedMap<Double, InterpolationRecord> exclusiveTailMap(double timestamp) {
        return m_poseBuffer.tailMap(timestamp, false);
    }

    /**
     * Sample the state estimate buffer.
     */
    @Override
    public SwerveModel get(double timestampSeconds) {
        return m_poseBuffer.get(timestampSeconds).m_state;
    }

    public Rotation2d getGyroOffset() {
        return m_gyroOffset;
    }

    /** Empty the buffer and add the given measurements. */
    public void reset(
            Rotation2d gyroAngle,
            SwerveModulePositions modulePositions,
            Pose2d pose,
            double timestampSeconds) {
        m_gyroOffset = pose.getRotation().minus(gyroAngle);

        // empty the buffer and add the current pose
        m_poseBuffer.reset(
                timestampSeconds,
                new InterpolationRecord(
                        m_kinodynamics.getKinematics(),
                        new SwerveModel(pose, new FieldRelativeVelocity(0, 0, 0)),
                        modulePositions));

        m_log_offset.log(() -> m_gyroOffset);
    }

    int size() {
        return m_poseBuffer.size();
    }

    double lastKey() {
        return m_poseBuffer.lastKey();
    }

}
