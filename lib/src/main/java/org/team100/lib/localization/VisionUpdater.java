package org.team100.lib.localization;

import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;

/**
 * Updates the pose buffer with any vision input that is waiting to be read, by
 * interpolating to find a pose for the vision timestamp, nudging that pose, and
 * then replaying all the later odometry.
 */
public class VisionUpdater implements VisionUpdaterInterface {

    private final SwerveDrivePoseEstimator100 m_estimator;
    /** For replay. */
    private final OdometryUpdater m_odometryUpdater;

    public VisionUpdater(
            SwerveDrivePoseEstimator100 estimator,
            OdometryUpdater odometryUpdater) {
        m_estimator = estimator;
        m_odometryUpdater = odometryUpdater;
    }

    /**
     * Put a new state estimate based on the supplied pose. If not current,
     * subsequent wheel updates are replayed.
     */
    @Override
    public void put(
            double timestampS,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma) {

        // Skip too-old measurement
        if (m_estimator.tooOld(timestampS)) {
            return;
        }

        // Sample the history at the measurement time.
        InterpolationRecord sample = m_estimator.getRecord(timestampS);

        // Nudge the sample towards the measurement.
        Pose2d nudged = VisionUpdater.nudge(
                sample.m_state.pose(), measurement, stateSigma, visionSigma);

        // Add the result
        m_estimator.put(
                timestampS,
                new SwerveModel(nudged, sample.m_state.velocity()),
                sample.m_wheelPositions);

        m_odometryUpdater.replay(timestampS);
    }

    /**
     * Nudge the sample towards the measurement.
     */
    public static Pose2d nudge(
            Pose2d sample,
            Pose2d measurement,
            double[] stateSigma,
            double[] visionSigma) {
        // Step 2: Measure the twist between the odometry pose and the vision pose.
        Twist2d twist = sample.log(measurement);
        // Step 4: Discount the twist based on the sigmas relative to each other
        Twist2d scaledTwist = getScaledTwist(stateSigma, visionSigma, twist);
        return sample.exp(scaledTwist);
    }

    public static Twist2d getScaledTwist(
            double[] stateSigma,
            double[] visionSigma,
            Twist2d twist) {
        // discount the vision update by this factor.
        final double[] K = getK(stateSigma, visionSigma);
        Twist2d scaledTwist = new Twist2d(
                K[0] * twist.dx,
                K[1] * twist.dy,
                K[2] * twist.dtheta);
        return scaledTwist;
    }

    public static double[] getK(double[] stateSigma, double[] visionSigma) {
        return new double[] {
                mix(Math.pow(stateSigma[0], 2), Math.pow(visionSigma[0], 2)),
                mix(Math.pow(stateSigma[1], 2), Math.pow(visionSigma[1], 2)),
                mix(Math.pow(stateSigma[2], 2), Math.pow(visionSigma[2], 2))
        };
    }

    /**
     * Given q and r stddev's, what mixture should that yield?
     * This is the "closed form Kalman gain for continuous Kalman filter with A = 0
     * and C = I. See wpimath/algorithms.md." ... but really it's just a mixer.
     * 
     * @param q state variance
     * @param r vision variance
     */
    public static double mix(final double q, final double r) {
        if (q == 0.0)
            return 0.0;
        return q / (q + Math.sqrt(q * r));
    }

}
