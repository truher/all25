package org.team100.lib.subsystems.swerve.commands.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelSE2;

/**
 * Adapts various types of manual drivers so `DriveManually` can switch between
 * them easily.
 */
public interface DriverAdapter {
    void apply(ModelSE2 s, Velocity t);

    void reset(ModelSE2 s);
}