package org.team100.lib.motion.servo;

import org.team100.lib.reference.Setpoints1d;

/**
 * Angular position control, e.g. for swerve steering axes or arm axes.
 * 
 * Servos no longer include profiles (in order to coordinate multiple axes), so
 * this just takes setpoints, and, for onboard versions, computes feedback
 * control.
 */
public interface AngularPositionServo {
    /**
     * Zeros controller errors, sets setpoint to current position.
     *
     * It is essential to call this after a period of disuse, to prevent transients.
     * 
     * To prevent oscillation, the previous setpoint is used to compute the profile,
     * but there needs to be an initial setpoint.
     */
    void reset();

    /**
     * Invalidates the current profile.
     */
    void setDutyCycle(double dutyCycle);

    void setTorqueLimit(double torqueNm);

    /**
     * Initializes the profile if necessary.
     * 
     * Sets the goal, updates the setpoint to the "next step" value towards it,
     * gives the setpoint to the outboard mechanism.
     * 
     * @param wrappedGoalRad The goal angle here wraps within [-pi, pi], using
     *                       output measurements, e.g. shaft radians, not motor
     *                       radians
     * @param torqueNm       feedforward for gravity or spring compensation
     */
    void setPositionProfiled(double wrappedGoalRad, double torqueNm);

    /**
     * Invalidates the current profile, sets the setpoint directly.
     * 
     * @param wrappedSetpoint both current and next setpoints so that the
     *                        implementation can choose the current one for feedback
     *                        and the next one for feedforward.
     * @param torqueNm        feedforward for gravity or spring compensation
     */
    void setPositionDirect(Setpoints1d wrappedSetpoint, double torqueNm);

    /**
     * This is the "wrapped" value, i.e. it is periodic within +/- pi.
     * 
     * Value should be updated in Robot.robotPeriodic().
     * 
     * @return Current position measurement, radians.
     */
    double getWrappedPositionRad();

    /** The "unwrapped" value domain is infinite. */
    double getUnwrappedPositionRad();

    /** Mechanism is following the desired setpoint. */
    boolean atSetpoint();

    boolean profileDone();

    /**
     * Profile is done, and we're on the setpoint.
     * 
     * Note this is affected by the setpoint update.
     * 
     * It really makes the most sense to call this *before* updating the setpoint,
     * because the measurement comes from the recent-past Takt and the updated
     * setpoint will be aiming at the next one.
     */
    boolean atGoal();

    void stop();

    void close();

    /** for logging */
    void periodic();
}
