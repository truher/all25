package org.team100.lib.reference.se2;

import org.team100.lib.state.ControlSE2;
import org.team100.lib.state.ModelSE2;

/** Always returns the same reference. */
public class ConstantReferenceSE2 implements ReferenceSE2 {
    private final ModelSE2 m_goal;

    public ConstantReferenceSE2(ModelSE2 goal) {
        m_goal = goal;
    }

    @Override
    public void initialize(ModelSE2 measurement) {
        //
    }

    @Override
    public ModelSE2 current() {
        return m_goal;
    }

    @Override
    public ControlSE2 next() {
        return m_goal.control();
    }

    @Override
    public boolean done() {
        return false;
    }

    @Override
    public ModelSE2 goal() {
        return m_goal;
    }

}
