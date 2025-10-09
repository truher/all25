package org.team100.lib.commands.drivetrain.manual;

import org.team100.lib.hid.Velocity;
import org.team100.lib.motion.drivetrain.state.GlobalSe2Velocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.math.MathUtil;

public interface FieldRelativeDriver {

    /**
     * @param state from the drivetrain
     * @param input control units [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    GlobalSe2Velocity apply(SwerveModel state, Velocity input);

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
    public static GlobalSe2Velocity scale(Velocity twist, double maxSpeed, double maxRot) {
        return new GlobalSe2Velocity(
                maxSpeed * MathUtil.clamp(twist.x(), -1, 1),
                maxSpeed * MathUtil.clamp(twist.y(), -1, 1),
                maxRot * MathUtil.clamp(twist.theta(), -1, 1));
    }

}
