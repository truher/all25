package org.team100.lib.subsystems.swerve.commands.manual;

import org.team100.lib.geometry.VelocitySE2;
import org.team100.lib.hid.Velocity;
import org.team100.lib.state.ModelSE2;

import edu.wpi.first.math.MathUtil;

public interface FieldRelativeDriver {

    /**
     * @param state from the drivetrain
     * @param input control units [-1,1]
     * @return feasible field-relative velocity in m/s and rad/s
     */
    VelocitySE2 apply(ModelSE2 state, Velocity input);

    void reset(ModelSE2 state);

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
    public static VelocitySE2 scale(Velocity twist, double maxSpeed, double maxRot) {
        return new VelocitySE2(
                maxSpeed * MathUtil.clamp(twist.x(), -1, 1),
                maxSpeed * MathUtil.clamp(twist.y(), -1, 1),
                maxRot * MathUtil.clamp(twist.theta(), -1, 1));
    }

}
