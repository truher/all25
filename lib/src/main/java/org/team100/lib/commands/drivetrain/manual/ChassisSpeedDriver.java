package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ChassisSpeedDriver  {

    ChassisSpeeds apply(SwerveModel state, DriverControl.Velocity input);

    void reset(SwerveModel state);
}
