package org.team100.frc2025.CalgamesArm;

import org.team100.lib.reference.r3.ReferenceR3;
import org.team100.lib.state.ControlR3;
import org.team100.lib.state.ModelR3;

/**
 * Like the drivetrain ReferenceController, but it does
 * outboard positional control, so it just passes through the "next" reference.
 */
public class CalgamesReferenceController {
    private static final boolean DEBUG = false;

    private final CalgamesMech m_subsystem;
    private final ReferenceR3 m_reference;

    public CalgamesReferenceController(CalgamesMech subsystem, ReferenceR3 reference) {
        m_subsystem = subsystem;
        m_reference = reference;
        m_reference.initialize(m_subsystem.getState());
    }

    public void execute() {
        try {
            ModelR3 measurement = m_subsystem.getState();
            ModelR3 current = m_reference.current();
            ModelR3 error = current.minus(measurement);
            if (DEBUG) {
                System.out.printf("error %s\n", error);
            }

            ControlR3 next = m_reference.next();
            m_subsystem.set(next);
        } catch (IllegalStateException ex) {
            // System.out.println(ex);
            // This happens when the trajectory generator produces an empty trajectory.
            // Ignore it for now.
        }
    }

    /**
     * Trajectory is complete and controller error is within tolerance.
     * 
     * Since positional feedback is outboard, the "at reference" thing is unknown.
     */
    public boolean isDone() {
        return m_reference.done();
    }

    /** Distance between the measurement and the goal. */
    public double toGo() {
        ModelR3 goal = m_reference.goal();
        ModelR3 measurement = m_subsystem.getState();
        return goal.minus(measurement).translation().getNorm();
    }

}
