package org.team100.lib.commands.r3;

import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SubsystemR3 extends Subsystem {
    /** Subsystem state for the current Takt. */
    SwerveModel getState();

    /** Apply the specified control, without moderation (e.g. limiting, profiles). */
    void set(SwerveControl control);

    /** Passthrough to motor stop. This is not "hold position", it is "disable". */
    void stop();
}
