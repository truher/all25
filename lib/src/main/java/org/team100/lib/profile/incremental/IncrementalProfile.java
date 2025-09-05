package org.team100.lib.profile.incremental;

import org.team100.lib.state.Control100;
import org.team100.lib.state.Model100;

/**
 * This profile takes incremental steps from the setpoint towards the goal.
 * 
 * One-dimensional, doesn't support coordination.
 */
public interface IncrementalProfile {
    /**
     * Return the control for dt in the future. The setpoint is a Control100 so that
     * we can regulate jerk.
     */
    Control100 calculate(double dt, Control100 setpoint, Model100 goal);

    /**
     * Find ETA by simply running the whole thing to the end.
     * You should use a coarse dt so that this doesn't take too long.
     * It's ok for the answer to be a little bit wrong.
     * 
     * The reasons we use a simulation approach are that all our profiles are short
     * so it's not that hard to do, some profiles can be hard to reason about
     * without simulating anyway, and it's guaranteed to work.
     */
    default double simulateForETA(double dt, Control100 initial, Model100 goal) {
        double t = 0;
        Model100 sample = initial.model();
        while (!sample.near(goal, 0.01)) {
            Control100 c = calculate(dt, sample.control(), goal);
            sample = c.model();
            t += dt;
            // TODO: configurable longest-ETA
            if (t > 10)
                return Double.POSITIVE_INFINITY;
        }
        return t;
    }

}
