package org.team100.frc2025.CalgamesArm;

import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;
import org.team100.lib.reference.SwerveReference;
import org.team100.lib.util.Util;

/**
 * Like the drivetrain ReferenceController, but it does
 * outboard positional control, so it just passes through the "next" reference.
 */
public class CalgamesReferenceController {
    private static final boolean DEBUG = false;

    private final CalgamesMech m_subsystem;
    private final SwerveReference m_reference;

    public CalgamesReferenceController(CalgamesMech subsystem, SwerveReference reference) {
        m_subsystem = subsystem;
        m_reference = reference;
    }

    public void execute() {
        SwerveModel measurement = m_subsystem.getState();
        SwerveModel current = m_reference.current();
        SwerveModel error = current.minus(measurement);
        if (DEBUG)
            Util.printf("error %s\n", error);

        SwerveControl next = m_reference.next();
        m_subsystem.set(next);
    }

    /**
     * Trajectory is complete and controller error is within tolerance.
     * 
     * Since positional feedback is outboard, the "at reference" thing is unknown.
     * 
     * TODO: make it known.
     * 
     */
    public boolean isDone() {
        return m_reference.done();// && m_controller.atReference();
    }

}
