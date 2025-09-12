package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.state.SwerveModuleStates;

public interface ModuleStateDriver {
    /**
     * @param input control units [-1,1]
     * @return module states
     */
    SwerveModuleStates apply(DriverControl.Velocity input);
}
