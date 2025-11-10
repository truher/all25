package org.team100.lib.subsystems.swerve.commands.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.subsystems.swerve.module.state.SwerveModuleStates;

public interface ModuleStateDriver {

    SwerveModuleStates apply(Velocity input);
}
