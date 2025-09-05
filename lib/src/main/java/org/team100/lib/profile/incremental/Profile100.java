package org.team100.lib.profile.incremental;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;
import org.team100.lib.util.Math100;
import org.team100.lib.util.Util;

/**
 * One-dimensional motion profile.
 * 
 * Use the ETA to coordinate multiple dimensions.
 */
public interface Profile100 extends IncrementalProfile {
    public static final boolean DEBUG = false;

    @FunctionalInterface
    interface ProfileMaker {
        /**
         * @param s parameter
         */
        Profile100 apply(double s);
    }

    /**
     * Return a new profile with acceleration scaled by s.
     */
    Profile100 scale(double s);

    /**
     * Find the scale factor that makes the profile complete in the specified time
     * (eta).
     */
    double solve(
            double dt,
            Control100 i,
            Model100 g,
            double eta,
            double etaTolerance);

    double getMaxVelocity();

    /**
     * Return scale factor to make the ETA equal to the desired ETA, by reducing
     * acceleration.
     * 
     * It never returns s > 1, and it also never scales more than 10X, i.e. never
     * returns s < 0.01.
     * 
     * It is very approximate, in order to not run too long. It's very primitive.
     */
    static double solveForSlowerETA(
            double dt,
            Control100 initial,
            Model100 goal,
            double eta,
            double sTolerance,
            ProfileMaker maker) {
        final double minS = 0.01;
        final double maxS = 1.0;
        double ss = Math100.findRoot(
                s -> getEtaDiff(dt, initial, goal, eta, s, maker),
                minS,
                getEtaDiff(dt, initial, goal, eta, minS, maker),
                maxS,
                getEtaDiff(dt, initial, goal, eta, maxS, maker),
                sTolerance,
                100);
        if (DEBUG)
            Util.printf("s %5.2f\n", ss);
        return ss;
    }

    /** Calculate the ETA from initial to goal, compare to expected. */
    static double getEtaDiff(
            double dt,
            Control100 initial,
            Model100 goal,
            double eta,
            double s,
            ProfileMaker maker) {
        Profile100 p = maker.apply(s);
        double etaS = p.simulateForETA(dt, initial, goal);
        return etaS - eta;
    }
}