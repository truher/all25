package org.team100.lib.subsystems.r3.commands.helper;

import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.r3.ReferenceR3;
import org.team100.lib.state.ControlR3;
import org.team100.lib.subsystems.r3.VelocitySubsystemR3;

/**
 * Actuates a velocity subsystem based on a reference.
 * 
 * The lifespan of this object is intended to be a single "playback" of a
 * trajectory, so create it in Command.initialize().
 */
public class VelocityReferenceControllerR3 extends ReferenceControllerR3Base {
    private final VelocitySubsystemR3 m_subsystem;

    /**
     * Call this from Command.initialize().
     */
    public VelocityReferenceControllerR3(
            LoggerFactory parent,
            VelocitySubsystemR3 subsystem,
            ControllerR3 controller,
            ReferenceR3 reference) {
        super(parent, subsystem, controller, reference);
        m_subsystem = subsystem;
    }

    @Override
    void execute100(ControlR3 next, GlobalVelocityR3 u) {
        m_subsystem.setVelocity(u);
    }
}
