package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.DriverControl;
import org.team100.lib.motion.drivetrain.SwerveModel;
import org.team100.lib.motion.drivetrain.kinodynamics.FieldRelativeVelocity;

import edu.wpi.first.math.MathUtil;

public interface FieldRelativeDriver {

    /**
     * @param state from the drivetrain
     * @param input control units [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    FieldRelativeVelocity apply(SwerveModel state, DriverControl.Velocity input);

    void reset(SwerveModel state);

    /**
     * Scales driver input to field-relative velocity.
     * 
     * This makes no attempt to address infeasibilty, it just multiplies.
     * 
     * @param twist    [-1,1]
     * @param maxSpeed meters per second
     * @param maxRot   radians per second
     * @return meters and rad per second as specified by speed limits
     */
    public static FieldRelativeVelocity scale(DriverControl.Velocity twist, double maxSpeed, double maxRot) {
        return new FieldRelativeVelocity(
                maxSpeed * MathUtil.clamp(twist.x(), -1, 1),
                maxSpeed * MathUtil.clamp(twist.y(), -1, 1),
                maxRot * MathUtil.clamp(twist.theta(), -1, 1));
    }

}
