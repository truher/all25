package org.team100.lib.motion.drivetrain;

import org.team100.lib.motion.drivetrain.state.GlobalSe2Velocity;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

import edu.wpi.first.wpilibj2.command.Subsystem;

/** For testing */
public interface DriveSubsystemInterface extends Subsystem {

    /** Scale and filter input before applying. */
    void driveInFieldCoords(GlobalSe2Velocity setpoint);

    /** No scaling or filtering. */
    void driveInFieldCoordsVerbatim(GlobalSe2Velocity setpoint);

    /** Drive state for the current Takt. */
    SwerveModel getState();

    /** Passthrough to motor stop. This is not "hold position", it is "disable". */
    void stop();

    /** Set the swerve limiter setpoint to the current velocity measurement. */
    void resetLimiter();
}