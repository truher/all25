package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ChassisSpeedDriver  {
    /**
     * @param input control units, [-1,1]
     * @return feasible chassis speeds in m/s and rad/s
     */

    ChassisSpeeds apply(SwerveModel state, DriverControl.Velocity input);

    void reset(SwerveModel state);

}
