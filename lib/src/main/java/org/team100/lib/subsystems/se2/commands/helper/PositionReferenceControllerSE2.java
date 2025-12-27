package org.team100.lib.subsystems.se2.commands.helper;

import org.team100.lib.controller.se2.ControllerSE2;
import org.team100.lib.controller.se2.NullControllerSE2;
import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.logging.LoggerFactory;
import org.team100.lib.reference.se2.ReferenceSE2;
import org.team100.lib.state.ControlSE2;
import org.team100.lib.subsystems.se2.PositionSubsystemSE2;

/**
 * Actuates a positional subsystem based on a reference.
 * 
 * Uses outboard positional control, so it just passes through the "next"
 * reference.
 * 
 * The lifespan of this object is intended to be a single "playback" of a
 * trajectory, so create it in Command.initialize().
 */
public class PositionReferenceControllerSE2 extends ReferenceControllerSE2Base {
    private final PositionSubsystemSE2 m_subsystem;

    /**
     * Call this from Command.initialize().
     */
    public PositionReferenceControllerSE2(
            LoggerFactory parent,
            PositionSubsystemSE2 subsystem,
            ReferenceSE2 reference) {
        super(parent, subsystem, nullController(parent), reference);
        m_subsystem = subsystem;
    }

    /**
     * @param next set the subsystem position to this
     * @param u    ignored, since this uses outboard feedback only.
     */
    @Override
    void execute100(ControlSE2 next, VelocitySE2 u) {
        m_subsystem.set(next);
    }

    /**
     * Doesn't control anything, just keeps track of atReference()
     */
    private static ControllerSE2 nullController(LoggerFactory parent) {
        return new NullControllerSE2(parent, 1, 1, 1, 1);
    }

}
