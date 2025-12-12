package org.team100.lib.profile.r3;

import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

public interface ProfileR3 {

    /**
     * For coordinated profiles, do whatever precalculation is required. *
     * 
     * @param i initial
     * @param g goal
     */
    void solve(ModelR3 i, ModelR3 g);

    /**
     * Compute the control for the end of the next time step.
     * 
     * @param i initial
     * @param g goal
     * @return control
     */
    ControlR3 calculate(ModelR3 i, ModelR3 g);

}