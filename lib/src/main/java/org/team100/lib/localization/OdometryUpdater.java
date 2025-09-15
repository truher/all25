package org.team100.lib.localization;

import java.util.Map.Entry;

import org.team100.lib.motion.drivetrain.kinodynamics.SwerveKinodynamics;
import org.team100.lib.motion.drivetrain.state.FieldRelativeDelta;
import org.team100.lib.motion.drivetrain.state.FieldRelativeVelocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.motion.drivetrain.state.SwerveModuleDeltas;
import org.team100.lib.motion.drivetrain.state.SwerveModulePositions;
import org.team100.lib.util.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Updates the pose buffer with new odometry by selecting the most-recent pose
 * and applying the pose delta represented by the odometry.
 */
public class OdometryUpdater {
    private static final boolean DEBUG = false;

    private final SwerveKinodynamics m_kinodynamics;
    private final SwerveDrivePoseEstimator100 m_poseEstimator;

    public OdometryUpdater(
            SwerveKinodynamics kinodynamics,
            SwerveDrivePoseEstimator100 estimator) {
        m_kinodynamics = kinodynamics;
        m_poseEstimator = estimator;
    }

    /**
     * Put a new state estimate based on gyro and wheel data. These are expected to
     * be current measurements -- there is no history replay here, though it won't
     * fail if you give it out-of-order input.
     * 
     * It Samples the history before the specified time, and records a new pose
     * based on the difference in wheel positions between the sample and the
     * specified positions.
     * 
     * The gyro angle overrides the odometry-derived gyro measurement, and
     * the gyro rate overrides the rate derived from the difference to the previous
     * state.
     */
    public void put(
            double currentTimeS,
            Rotation2d gyroAngleRadNWU,
            double gyroRateRad_SNWU,
            SwerveModulePositions wheelPositions) {

        // the entry right before this one, the basis for integration.
        Entry<Double, InterpolationRecord> lowerEntry = m_poseEstimator.lowerEntry(
                currentTimeS);

        if (lowerEntry == null) {
            // Util.println("lower entry is null");
            // We're at the beginning. There's nothing to apply the wheel position delta to.
            // This should never happen.
            return;
        }

        double dt = currentTimeS - lowerEntry.getKey();
        InterpolationRecord value = lowerEntry.getValue();
        SwerveModel previousState = value.m_state;
        if (DEBUG)
            Util.printf("previous x %.6f y %.6f\n", previousState.pose().getX(), previousState.pose().getY());

        SwerveModuleDeltas modulePositionDelta = SwerveModuleDeltas.modulePositionDelta(
                value.m_wheelPositions,
                wheelPositions);
        if (DEBUG)
            Util.printf("modulePositionDelta %s\n", modulePositionDelta);

        Twist2d twist = m_kinodynamics.getKinematics().toTwist2d(modulePositionDelta);
        if (DEBUG)
            Util.printf("twist x %.6f y %.6f theta %.6f\n", twist.dx, twist.dy, twist.dtheta);
        // replace the twist dtheta with one derived from the current pose
        // pose angle based on the gyro (which is more accurate)

        Rotation2d angle = gyroAngleRadNWU.plus(m_poseEstimator.getGyroOffset());
        if (DEBUG)
            Util.printf("angle %.6f\n", angle.getRadians());
        twist.dtheta = angle.minus(previousState.pose().getRotation()).getRadians();

        Pose2d newPose = previousState.pose().exp(twist);
        if (DEBUG)
            Util.printf("new pose x %.6f y %.6f\n", newPose.getX(), newPose.getY());

        // this is the backward finite difference velocity from odometry
        FieldRelativeDelta odoVelo = FieldRelativeDelta.delta(
                previousState.pose(), newPose)
                .div(dt);

        // use the gyro rate instead of the odometry-derived rate
        FieldRelativeVelocity velocity = new FieldRelativeVelocity(
                odoVelo.getX(),
                odoVelo.getY(),
                gyroRateRad_SNWU);

        SwerveModel swerveState = new SwerveModel(newPose, velocity);

        m_poseEstimator.put(currentTimeS, swerveState, wheelPositions);
    }

}
