package org.team100.lib.subsystems.r3.commands.helper;

import org.team100.lib.controller.r3.ControllerR3;
import org.team100.lib.controller.r3.NullControllerR3;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.r3.ReferenceR3;
import org.team100.lib.state.ControlR3;
import org.team100.lib.subsystems.r3.PositionSubsystemR3;

/**
 * Actuates a positional subsystem based on a reference.
 * 
 * Uses outboard positional control, so it just passes through the "next"
 * reference.
 * 
 * The lifespan of this object is intended to be a single "playback" of a
 * trajectory, so create it in Command.initialize().
 */
public class PositionReferenceControllerR3 extends ReferenceControllerR3Base {
    private final PositionSubsystemR3 m_subsystem;

    /**
     * Call this from Command.initialize().
     */
    public PositionReferenceControllerR3(
            LoggerFactory parent,
            PositionSubsystemR3 subsystem,
            ReferenceR3 reference) {
        super(parent, subsystem, nullController(parent), reference);
        m_subsystem = subsystem;
    }

    /**
     * @param next set the subsystem position to this
     * @param u    ignored, since this uses outboard feedback only.
     */
    @Override
    void execute100(ControlR3 next, VelocitySE2 u) {
        m_subsystem.set(next);
    }

    /**
     * Doesn't control anything, just keeps track of atReference()
     */
    private static ControllerR3 nullController(LoggerFactory parent) {
        return new NullControllerR3(parent, 1, 1, 1, 1);
    }

}
