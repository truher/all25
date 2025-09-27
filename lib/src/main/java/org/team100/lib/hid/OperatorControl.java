package org.team100.lib.hid;

import org.team100.lib.hid.DriverControl.Velocity;

/**
 * Represents the HID used by the "operator" role, which typically controls
 * everything other than the drivetrain.
 * 
 * When you change this interface, you'll need to also change the "proxy," don't
 * forget. :-)
 */
public interface OperatorControl {

    default String getHIDName() {
        return "No HID Found!!";
    }

    /** For "flying" the arm around manually. */
    default Velocity velocity() {
        return new Velocity(0, 0, 0);
    }

    default boolean manual() {
        return false;
    }

    ////////////////////////////////////////////////////////////
    //
    // CLIMB
    //

    /**
     * Activate manual control of winch duty cycle.
     * For resetting the climber position.
     */
    default boolean activateManualClimb() {
        return false;
    }

    /**
     * Set winch duty cycle directly.
     * For resetting the climber position.
     */
    default double manualClimbSpeed() {
        return 0.0;
    }

    /**
     * Extend and spin climber for intake.
     */
    default boolean climbIntake() {
        return false;
    }

    //
    //
    //

    default boolean intake() {
        return false;
    }

    default boolean outtake() {
        return false;
    }
}
