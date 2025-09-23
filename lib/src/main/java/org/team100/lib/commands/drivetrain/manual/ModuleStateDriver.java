package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.state.SwerveModuleStates;

public interface ModuleStateDriver {

    SwerveModuleStates apply(DriverControl.Velocity input);
}
