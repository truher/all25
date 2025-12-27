package org.team100.lib.subsystems.se2.commands.helper;

import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.se2.ReferenceSE2;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.subsystems.se2.VelocitySubsystemSE2;

/**
 * Actuates a velocity subsystem based on a reference.
 * 
 * The lifespan of this object is intended to be a single "playback" of a
 * trajectory, so create it in Command.initialize().
 */
public class VelocityReferenceControllerSE2 extends ReferenceControllerSE2Base {
    private final VelocitySubsystemSE2 m_subsystem;

    /**
     * Call this from Command.initialize().
     */
    public VelocityReferenceControllerSE2(
            LoggerFactory parent,
            VelocitySubsystemSE2 subsystem,
            ControllerSE2 controller,
            ReferenceSE2 reference) {
        super(parent, subsystem, controller, reference);
        m_subsystem = subsystem;
    }

    /**
     * @param next ignored
     * @param u    represents the control for the next dt, so it's also what the
     *             subsystem should be doing at the next timestep. set the subsystem
     *             velocity to this
     */
    @Override
    void execute100(ControlSE2 next, VelocitySE2 u) {
        m_subsystem.setVelocity(u);
    }
}
