package org.team100.lib.hid;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents the HID used by the "driver" role, which typically focuses on
 * controlling the drivetrain only.
 * 
 * Implementations should do their own deadbanding, scaling, expo, etc.
 * 
 * The intention is for this interface to represent the control, not to
 * represent the control's effect on the robot. So, for example, velocity inputs
 * are scaled to control units, ([-1,1]), not robot units (m/s).
 */
public interface DriverControl {
    /**
     * This represents driver's velocity command, usually mapped to three axes in
     * the control, so the ranges are [-1,1]
     */
    public static record Velocity(double x, double y, double theta) {

        /**
         * Clip the translational velocity to the unit circle.
         * 
         * This means that there will be no stick response outside the circle, but the
         * inside will be unchanged.
         * 
         * The argument for clipping is that it leaves the response inside the circle
         * alone: with squashing, the diagonals are more sensitive.
         * 
         * The argument for squashing is that it preserves relative response in
         * the corners: with clipping, going full speed diagonally and then "slowing
         * down a little" will do nothing.
         * 
         * If you'd like to avoid clipping, then squash the input upstream, in the
         * control class.
         */
        public DriverControl.Velocity clip(double maxMagnitude) {
            double hyp = Math.hypot(x(), y());
            if (hyp < 1e-12)
                return this;
            double clamped = Math.min(hyp, maxMagnitude);
            double ratio = clamped / hyp;
            return new DriverControl.Velocity(ratio * x(), ratio * y(), theta());
        }
    }

    public enum Speed {
        SLOW,
        MEDIUM,
        NORMAL
    }

    default String getHIDName() {
        return "No HID Found!!";
    }

    /**
     * Proportional robot velocity control.
     * 
     * Forward positive, left positive, counterclockwise positive, [-1,1]
     *
     * The expectation is that the control interval, [-1,1] will be transformed to
     * robot motion in some simple way.
     * 
     * There are two aspects of this transformation that are not simple:
     * 
     * 1. Controls should expect translational input outside the unit circle to be
     * clipped. If you'd like to avoid that, then squash the input in the control
     * class.
     * 
     * 2. Translation and rotation conflict: in reality, full translation speed
     * implies zero rotation speed, and vice versa. The SwerveSetpointGenerator and
     * the SwerveDriveKinemataics desaturator address this issue, though at a lower
     * level.
     */
    default Velocity velocity() {
        return new Velocity(0, 0, 0);
    }

    /**
     * Absolute rotational input. This could be mapped to a turntable or a POV hat
     * or similar. Return null if the control should be ignored.
     */
    default Rotation2d desiredRotation() {
        return null;
    }

    /**
     * Reset the pose estimator's gyro offset so that the current gyro rotation
     * corresponds to zero.
     */
    default boolean resetRotation0() {
        return false;
    }

    /**
     * Reset the pose estimator's gyro offset so that the current gyro rotation
     * corresponds to 180.
     */
    default boolean resetRotation180() {
        return false;
    }

    /**
     * Drive to a scoring location at the reef. The specific location
     * is chosen by the operator.
     */
    default boolean toReef() {
        return false;
    }

    default boolean feedFunnel() {
        return false;
    }

    /** Aim the front of the robot at the reef center. */
    default boolean useReefLock() {
        return false;
    }

    /** Aim the back of the robot at the nearest station. */
    default boolean driveWithBargeAssist() {
        return false;
    }

    ////////////////////////////////////////////////////////////
    //
    // CLIMB
    //
    /**
     * Pull climber in and drive forward.
     */
    default boolean climb() {
        return false;
    }

    ////////////////////////////////////////////////////////////
    //
    // These are for prototyping with Xbox controllers.
    // Please don't use these for comp.

    default boolean test() {
        return false;
    }

    default boolean button4() {
        return false;
    }

    default boolean button5() {
        return false;
    }

    default boolean x() {
        return false;
    }

    default boolean y() {
        return false;
    }

    default boolean a() {
        return false;
    }

    default boolean b() {
        return false;
    }

}
