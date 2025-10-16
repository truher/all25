package org.team100.lib.motion.drivetrain;

import org.team100.lib.geometry.GlobalVelocityR3;
import org.team100.lib.state.ModelR3;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** For testing */
public interface DriveSubsystemInterface extends Subsystem {

    /** Scale and filter input before applying. */
    void driveInFieldCoords(GlobalVelocityR3 setpoint);

    /** No scaling or filtering. */
    void driveInFieldCoordsVerbatim(GlobalVelocityR3 setpoint);

    /** Drive state for the current Takt. */
    ModelR3 getState();

    /** Passthrough to motor stop. This is not "hold position", it is "disable". */
    void stop();

    /** Set the swerve limiter setpoint to the current velocity measurement. */
    void resetLimiter();
}