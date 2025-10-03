package org.team100.lib.reference;

import org.team100.lib.motion.drivetrain.state.SwerveControl;
import org.team100.lib.motion.drivetrain.state.SwerveModel;

/** A source of references for drivetrain control. */
public interface SwerveReference {

    /**
     * Make current() return the starting point, which might be informed by the
     * supplied measurement, or not.
     */
    void initialize(SwerveModel measurement);

    /**
     * Reference for the current time. This can be null (e.g. between instantiation
     * and initialization).
     */
    SwerveModel current();

    /**
     * Reference for 0.02 sec in the future. This can be null (e.g. between
     * instantiation and initialization).
     */
    SwerveControl next();

    /**
     * The reference series has reached the end, and will keep returning the same
     * (goal) reference.
     */
    boolean done();

    /**
     * For reference sources that have endpoints, what is the endpoint?
     */
    SwerveModel goal();
}
