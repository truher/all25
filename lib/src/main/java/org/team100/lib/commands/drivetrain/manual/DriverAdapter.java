package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

/**
 * Adapts various types of manual drivers so `DriveManually` can switch between
 * them easily.
 */
public interface DriverAdapter {
    void apply(SwerveModel s, DriverControl.Velocity t);

    void reset(SwerveModel s);
}