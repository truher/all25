package org.team100.lib.controller.drivetrain;

import org.team100.lib.motion.drivetrain.state.GlobalSe2Velocity;
import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

/**
 * Feedback and feedforward control.
 */
public interface SwerveController  {

    /**
     * Feedback should compare the current-instant measurement to the
     * current-instant reference.
     * 
     * Feedforward should be looking at the next-step reference.
     * 
     * Previous versions of this method used a single reference for both.
     * 
     * @param measurement      Current measurement state in field coordinates
     * @param currentReference Current reference state i.e. setpoint
     * @param nextReference    Reference for dt in the future, used for feedforward.
     * @return Control output, should be given to
     *         SwerveDriveSubsystem.driveInFieldCoords() or something similar.
     */
    GlobalSe2Velocity calculate(
            SwerveModel measurement,
            SwerveModel currentReference,
            SwerveControl nextReference);

    /**
     * True if the error is within tolerance of the reference. The definitions of
     * "error" and "tolerance" here are implementation-specific.
     */
    boolean atReference();

    /**
     * Reset controller state, e.g. velocity error.
     */
    void reset();

}
