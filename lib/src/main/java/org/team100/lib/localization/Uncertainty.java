package org.team100.lib.localization;

import org.team100.lib.experiments.Experiment;
import org.team100.lib.experiments.Experiments;

import edu.wpi.first.math.geometry.Twist2d;

/**
 * Methods governing vision update uncertainties.
 */
public class Uncertainty {
    /** this is the default value which, in hindsight, seems ridiculously high. */
    private static final double[] DEFAULT_STATE_STDDEV = new double[] {
            0.1,
            0.1,
            0.1 };

    /**
     * This value is tuned so that errors scale at 0.2x per second. See
     * SwerveDrivePoseEstimator100Test::testFirmerNudge.
     */
    static final double[] TIGHT_STATE_STDDEV = new double[] {
            0.001,
            0.001,
            0.1 };

    /** This is an educated guess. */
    static double[] visionMeasurementStdDevs(double distanceM) {
        if (Experiments.instance.enabled(Experiment.AvoidVisionJitter)) {
            /*
             * NEW (3/12/25), 2 cm std dev seems kinda realistic for 1 m.
             * 
             * If it still jitters, try 0.03 or 0.05, but watch out for slow convergence.
             */
            final double K = 0.03;
            return new double[] {
                    (K * distanceM) + 0.01,
                    (K * distanceM) + 0.01,
                    Double.MAX_VALUE };
        }
        /*
         * Standard deviation of pose estimate, as a fraction of target range.
         * This is a guess based on figure 5 in the Apriltag2 paper:
         * https://april.eecs.umich.edu/media/media/pdfs/wang2016iros.pdf
         */
        final double K = 0.03;
        return new double[] {
                K * distanceM,
                K * distanceM,
                Double.MAX_VALUE };
    }

    static double[] stateStdDevs() {
        if (Experiments.instance.enabled(Experiment.AvoidVisionJitter)) {
            return Uncertainty.TIGHT_STATE_STDDEV;
        }
        return Uncertainty.DEFAULT_STATE_STDDEV;
    }

    static Twist2d getScaledTwist(
            double[] stateSigma,
            double[] visionSigma,
            Twist2d twist) {
        // discount the vision update by this factor.
        final double[] K = Uncertainty.getK(stateSigma, visionSigma);
        Twist2d scaledTwist = new Twist2d(
                K[0] * twist.dx,
                K[1] * twist.dy,
                K[2] * twist.dtheta);
        return scaledTwist;
    }

    static double[] getK(double[] stateSigma, double[] visionSigma) {
        return new double[] {
                Uncertainty.mix(Math.pow(stateSigma[0], 2), Math.pow(visionSigma[0], 2)),
                Uncertainty.mix(Math.pow(stateSigma[1], 2), Math.pow(visionSigma[1], 2)),
                Uncertainty.mix(Math.pow(stateSigma[2], 2), Math.pow(visionSigma[2], 2))
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
    static double mix(final double q, final double r) {
        if (q == 0.0)
            return 0.0;
        return q / (q + Math.sqrt(q * r));
    }

}
