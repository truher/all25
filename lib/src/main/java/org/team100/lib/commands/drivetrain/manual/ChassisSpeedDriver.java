package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ChassisSpeedDriver  {

    ChassisSpeeds apply(SwerveModel state, Velocity input);

    void reset(SwerveModel state);
}
