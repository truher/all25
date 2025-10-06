package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

/**
 * Adapts various types of manual drivers so `DriveManually` can switch between
 * them easily.
 */
public interface DriverAdapter {
    void apply(SwerveModel s, Velocity t);

    void reset(SwerveModel s);
}