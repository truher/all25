package org.team100.lib.subsystems.swerve.commands.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ChassisSpeedDriver  {

    ChassisSpeeds apply(ModelSE2 state, Velocity input);

    void reset(ModelSE2 state);
}
