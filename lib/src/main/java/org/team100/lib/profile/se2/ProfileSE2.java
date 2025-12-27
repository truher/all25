package org.team100.lib.profile.se2;

import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;

public interface ProfileSE2 {

    /**
     * For coordinated profiles, do whatever precalculation is required. *
     * 
     * @param i initial
     * @param g goal
     */
    void solve(ModelSE2 i, ModelSE2 g);

    /**
     * Compute the control for the end of the next time step.
     * 
     * @param i initial
     * @param g goal
     * @return control
     */
    ControlSE2 calculate(ModelSE2 i, ModelSE2 g);

}